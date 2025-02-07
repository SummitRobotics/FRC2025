# Check if .env file exists
if (!(Test-Path ".env")) {
    Write-Error "No .env file found"
    exit 1
}

Write-Host "Reading .env file..."

# Read and parse .env file
Get-Content ".env" | ForEach-Object {
    if ($_ -match '^([^#][^=]+)=(.+)$') {
        $key = $matches[1].Trim()
        $value = $matches[2].Trim()

        Write-Host "Processing: $key = $value"

        # Set permanent environment variable (elevated)
        # Start-Process powershell -Verb runAs -ArgumentList "setx $key `"$value`"" -Wait
        Start-Process powershell -ArgumentList "setx $key `"$value`""

        # Set for current session
        [Environment]::SetEnvironmentVariable($key, $value, "Process")

        # Verify using GetEnvironmentVariable
        $currentValue = [Environment]::GetEnvironmentVariable($key, "Process")
        Write-Host "Environment variable $key is set to: $currentValue"
    }
}

Write-Host "Environment variables set successfully"