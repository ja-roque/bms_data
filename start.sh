#!/bin/bash
# Start script for BMS Dashboard

echo "Starting BMS Dashboard Server..."
echo "You can access the dashboard at http://localhost:4567"
echo "Press Ctrl+C to stop the server"
echo ""

# Check if Ruby is installed
if ! command -v ruby &> /dev/null; then
    echo "Ruby is not installed. Please install Ruby before running this application."
    exit 1
fi

# Check if Bundler is installed
if ! command -v bundle &> /dev/null; then
    echo "Bundler is not installed. Installing bundler..."
    gem install bundler
fi

# Install dependencies if they aren't already
if [ ! -d "vendor/bundle" ]; then
    echo "Installing required gems..."
    bundle install --path vendor/bundle
fi

# Run the application
bundle exec ruby app.rb 