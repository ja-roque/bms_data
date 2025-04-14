require 'pg'

def setup_database
  # Connect to default postgres database to create our database
  conn = PG.connect(dbname: 'postgres')
  
  # Create database if it doesn't exist
  begin
    conn.exec("CREATE DATABASE bms_data")
    puts "Created database bms_data"
  rescue PG::Error => e
    if e.message.include?("already exists")
      puts "Database bms_data already exists"
    else
      puts "Error creating database: #{e.message}"
      exit 1
    end
  end

  # Connect to our new database
  conn = PG.connect(dbname: 'bms_data')

  # Create tables
  conn.exec <<-SQL
    CREATE TABLE IF NOT EXISTS bms_readings (
      id SERIAL PRIMARY KEY,
      device_name VARCHAR(255),
      voltage FLOAT,
      current FLOAT,
      power FLOAT,
      soc INTEGER,
      capacity FLOAT,
      temperature FLOAT[],
      protection_status JSONB,
      port_status JSONB,
      timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );

    -- Create index for faster queries by device and time
    CREATE INDEX IF NOT EXISTS idx_bms_readings_device_time 
    ON bms_readings(device_name, timestamp);
  SQL

  puts "Database setup complete"
rescue PG::Error => e
  puts "Error setting up database: #{e.message}"
  exit 1
ensure
  conn&.close
end

setup_database 