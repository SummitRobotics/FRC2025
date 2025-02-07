#!/bin/bash

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo "Error: No .env file found"
    exit 1
fi

echo "Reading .env file..."

# Read and parse .env file
while IFS= read -r line || [ -n "$line" ]; do
    # Skip comments and empty lines
    [[ $line =~ ^#.*$ || -z $line ]] && continue

    # Extract key and value
    if [[ $line =~ ^([^=]+)=(.*)$ ]]; then
        key="${BASH_REMATCH[1]}"
        value="${BASH_REMATCH[2]}"

        # Trim whitespace
        key="${key// /}"
        value="${value// /}"

        echo "Processing: $key = $value"

        # Export for current session only
        export "$key=$value"

        # Verify
        echo "Environment variable $key is set to: ${!key}"
    fi
done < ".env"

echo "Environment variables set successfully"
