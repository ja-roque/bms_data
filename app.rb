require 'sinatra'
require 'sinatra/json'
require 'json'
require 'time'
require 'mqtt'
require 'sequel'
require 'dotenv'

# Load environment variables
Dotenv.load

# Enable Sequel postgres extensions
Sequel.extension :pg_array

class MQTTServer < Sinatra::Base
  set :public_folder, File.dirname(__FILE__) + '/public'
  set :views, File.dirname(__FILE__) + '/views'
  set :port, ENV['PORT'] || 4567
  set :bind, ENV['HOST'] || '0.0.0.0'
  set :server, 'thin'
  set :connections, []

  # Store received BMS data
  $bms_data = []
  $last_update = nil
  $max_stored_entries = 100
  $last_device_updates = {} # Track last update per device

  class << self
    attr_accessor :db
  end

  # Initialize database connection
  def initialize
    super
    self.class.db ||= Sequel.connect(
      adapter: 'postgres',
      host: ENV['DB_HOST'],
      port: ENV['DB_PORT'],
      database: ENV['DB_NAME'],
      user: ENV['DB_USER'],
      password: ENV['DB_PASSWORD'],
      sslmode: ENV['DB_SSLMODE']
    )

    # Clean up old data (older than 1 month)
    self.class.db[:bms_readings].where(Sequel.lit('timestamp < NOW() - INTERVAL \'1 month\'')).delete
  end

  def self.store_data(data)
    device_name = data.dig('name')
    return unless device_name

    current_time = Time.now.to_f
    last_update = $last_device_updates[device_name] || 0
    
    # Only store if 5 minutes have passed for this specific device
    return if (current_time - last_update) < 300

    puts "Processing device: #{device_name}"
    puts "Last update was: #{Time.at(last_update)}"
    puts "Current time is: #{Time.at(current_time)}"

    # Convert temperature array to proper format
    temps = data.dig('basicInfo', 't')&.to_a&.map(&:to_f)
    temps = Sequel.pg_array(temps) if temps

    begin
      self.db[:bms_readings].insert(
        device_name: device_name,
        voltage: data.dig('basicInfo', 'v'),
        current: data.dig('basicInfo', 'i'),
        power: data.dig('basicInfo', 'p'),
        soc: data.dig('basicInfo', 'soc'),
        capacity: data.dig('basicInfo', 'cap'),
        temperature: temps,
        protection_status: data.dig('basicInfo', 'prot')&.to_json,
        port_status: data.dig('basicInfo', 'ports')&.to_json,
        timestamp: Time.at(current_time)
      )
      $last_device_updates[device_name] = current_time
      puts "Successfully stored data for #{device_name}"
    rescue => e
      puts "Error storing data for #{device_name}: #{e.message}"
    end
  end

  get '/' do
    erb :dashboard
  end

  get '/data' do
    content_type 'text/event-stream'
    stream(:keep_open) do |out|
      settings.connections << out
      out.callback { settings.connections.delete(out) }
    end
  end

  Thread.new do
    MQTT::Client.connect(
      host: ENV['MQTT_HOST'],
      port: ENV['MQTT_PORT'].to_i,
      username: ENV['MQTT_USERNAME'],
      password: ENV['MQTT_PASSWORD']
    ) do |client|
      client.get(ENV['MQTT_TOPIC']) do |topic, message|
        begin
          # Validate JSON before parsing
          next unless message.strip.start_with?('{') && message.strip.end_with?('}')
          
          data = JSON.parse(message)
          next unless data.is_a?(Hash) # Ensure we have a valid JSON object
          
          data['timestamp'] = Time.now.iso8601
          $last_update = Time.now
          
          # Store data with a limit on number of entries
          $bms_data.unshift(data)
          if $bms_data.length > $max_stored_entries
            $bms_data.pop
          end
          
          # Store data in database every 5 minutes
          if data['devices']&.is_a?(Array)
            data['devices'].each do |device|
              next unless device.is_a?(Hash) && device['name'] # Ensure device data is valid
              MQTTServer.store_data(device)
            end
          end
          
          settings.connections.each do |out|
            out << "data: #{message}\n\n"
          end
        rescue JSON::ParserError => e
          puts "Ignoring malformed JSON data: #{e.message}"
        rescue StandardError => e
          puts "Error processing message: #{e.message}"
          puts e.backtrace
        end
      end
    end
  end

  # Helper method to check if we have any data
  def has_data?
    !$bms_data.empty?
  end

  get '/api/energy' do
    content_type :json
    
    # Get optional time range parameters (default to last 24 hours)
    start_time = params[:start] ? Time.parse(params[:start]) : (Time.now - 86400)
    end_time = params[:end] ? Time.parse(params[:end]) : Time.now
    
    # Get all unique device names
    devices = self.class.db[:bms_readings]
      .select(:device_name)
      .distinct
      .order(:device_name)
      .map(:device_name)

    # Get data for all devices
    device_data = devices.map do |device_name|
      # Use AT TIME ZONE to convert timestamps in the database query
      readings = self.class.db[:bms_readings]
        .where(device_name: device_name)
        .where(Sequel.lit('timestamp BETWEEN ? AND ?', start_time, end_time))
        .order(:timestamp)
        .select(
          Sequel.lit("timestamp AT TIME ZONE 'UTC' AT TIME ZONE 'America/Tegucigalpa' as timestamp"),
          :voltage,
          :current,
          :power,
          :soc,
          Sequel.lit("temperature[1] as temp1")
        )
        .all

      # Process readings
      processed_data = readings.map do |reading|
        {
          # Format the timestamp that's already in Honduras time
          timestamp: reading[:timestamp].strftime('%Y-%m-%dT%H:%M:%S%:z'),
          voltage: reading[:voltage],
          current: reading[:current],
          power: reading[:power],
          soc: reading[:soc],
          temp1: reading[:temp1] ? (reading[:temp1] - 273.15).round(2) : nil
        }
      end

      {
        device_name: device_name,
        data_points: processed_data
      }
    end

    # Convert start and end times to Honduras time
    honduras_start = start_time.getlocal('-06:00')
    honduras_end = end_time.getlocal('-06:00')

    {
      start_time: honduras_start.iso8601,
      end_time: honduras_end.iso8601,
      devices: device_data
    }.to_json
  end

  get '/energy' do
    erb :energy_dashboard
  end
end

# Run the server if this file is executed directly
MQTTServer.run! if __FILE__ == $0 