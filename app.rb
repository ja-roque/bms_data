require 'sinatra'
require 'sinatra/json'
require 'json'
require 'time'
require 'mqtt'
require 'pg'
require 'dotenv'

# Load environment variables
Dotenv.load

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

  # Initialize database connection
  def initialize
    super
    @db = PG.connect(
      host: ENV['DB_HOST'],
      port: ENV['DB_PORT'],
      dbname: ENV['DB_NAME'],
      user: ENV['DB_USER'],
      password: ENV['DB_PASSWORD'],
      sslmode: ENV['DB_SSLMODE']
    )
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
        data = JSON.parse(message)
        settings.connections.each do |out|
          out << "data: #{message}\n\n"
        end
      end
    end
  end

  # Helper method to check if we have any data
  def has_data?
    !$bms_data.empty?
  end
end

# Run the server if this file is executed directly
MQTTServer.run! if __FILE__ == $0 