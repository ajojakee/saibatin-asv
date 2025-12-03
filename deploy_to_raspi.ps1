param(
    [string]$RaspiHost = "10.227.177.110",  # ← FIXED! IP Raspberry Pi yang benar
    [string]$User = "pi",
    [string]$ServerIP = "10.227.177.157"    # IP laptop teman Anda
)

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "   SAIBATIN - Deploy to Raspberry Pi" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Server IP: $ServerIP" -ForegroundColor Yellow
Write-Host "Raspberry Pi: $RaspiHost" -ForegroundColor Yellow
Write-Host "User: $User" -ForegroundColor Yellow
Write-Host ""

# Test connection
Write-Host "Testing connection to $RaspiHost..." -ForegroundColor Yellow
$pingResult = Test-Connection -ComputerName $RaspiHost -Count 2 -Quiet -ErrorAction SilentlyContinue

if (-not $pingResult) {
    Write-Host "ERROR: Cannot reach Raspberry Pi at $RaspiHost" -ForegroundColor Red
    Write-Host ""
    Write-Host "Troubleshooting:" -ForegroundColor Yellow
    Write-Host "  1. Raspberry Pi is powered on" -ForegroundColor White
    Write-Host "  2. Connected to same WiFi network (10.227.177.x)" -ForegroundColor White
    Write-Host "  3. Try: ssh ${User}@${RaspiHost}" -ForegroundColor White
    Write-Host ""
    Write-Host "Alternative:" -ForegroundColor Cyan
    Write-Host "  Find IP: arp -a | Select-String '10.227'" -ForegroundColor White
    Write-Host "  Then run: .\deploy_to_raspi.ps1 -RaspiHost <IP>" -ForegroundColor White
    exit 1
}

Write-Host "✓ Connection OK" -ForegroundColor Green
Write-Host ""

# Create remote directory
Write-Host "Creating remote directory..." -ForegroundColor Yellow
ssh "${User}@${RaspiHost}" "mkdir -p /home/pi/saibatin-asv-main/captures"
Write-Host "✓ Directory created" -ForegroundColor Green
Write-Host ""

# Upload files
Write-Host "Uploading files to Raspberry Pi..." -ForegroundColor Yellow
Write-Host ""

$filesToUpload = @(
    @{File="saibatin_production_fixed.py"; Required=$true},
    @{File="mission_state_machine.py"; Required=$false},
    @{File="server.js"; Required=$false},
    @{File="index.html"; Required=$false},
    @{File="package.json"; Required=$false},
    @{File="test_pixhawk.py"; Required=$false},
    @{File="test_brica_capture.py"; Required=$false},
    @{File="calibrate_hsv.py"; Required=$false},
    @{File="setup_brica_wifi.sh"; Required=$false},
    @{File="README_BRICA_SETUP.md"; Required=$false}
)

$uploadedCount = 0
$failedCount = 0

foreach ($item in $filesToUpload) {
    $file = $item.File
    if (Test-Path $file) {
        Write-Host "  Uploading: $file" -ForegroundColor Cyan -NoNewline
        scp $file "${User}@${RaspiHost}:/home/pi/saibatin-asv-main/" 2>$null
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host " ✓" -ForegroundColor Green
            $uploadedCount++
        } else {
            Write-Host " ✗" -ForegroundColor Red
            $failedCount++
            if ($item.Required) {
                Write-Host "  ERROR: Required file failed to upload!" -ForegroundColor Red
            }
        }
    } else {
        Write-Host "  Skipped: $file (not found)" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "Upload summary: $uploadedCount succeeded, $failedCount failed" -ForegroundColor Cyan
Write-Host ""

# Update server IP in Python script
Write-Host "Updating server IP to $ServerIP..." -ForegroundColor Yellow
$sedCommand = "sed -i 's|SOCKETIO_SERVER = \"http://.*:5000\"|SOCKETIO_SERVER = \"http://${ServerIP}:5000\"|g' /home/pi/saibatin-asv-main/saibatin_production_fixed.py"
ssh "${User}@${RaspiHost}" $sedCommand
Write-Host "✓ Server IP updated" -ForegroundColor Green
Write-Host ""

# Install dependencies
Write-Host "Installing Python dependencies..." -ForegroundColor Yellow
Write-Host "(This may take several minutes on first run)" -ForegroundColor Gray
ssh "${User}@${RaspiHost}" "pip3 install opencv-python numpy python-socketio pymavlink requests 2>&1 | tail -n 3"
Write-Host "✓ Dependencies checked" -ForegroundColor Green
Write-Host ""

# Make scripts executable
Write-Host "Setting permissions..." -ForegroundColor Yellow
ssh "${User}@${RaspiHost}" "chmod +x /home/pi/saibatin-asv-main/*.py 2>/dev/null"
ssh "${User}@${RaspiHost}" "chmod +x /home/pi/saibatin-asv-main/*.sh 2>/dev/null"
Write-Host "✓ Permissions set" -ForegroundColor Green
Write-Host ""

# Verify deployment
Write-Host "Verifying deployment..." -ForegroundColor Yellow
$fileCount = ssh "${User}@${RaspiHost}" "ls -1 /home/pi/saibatin-asv-main/*.py 2>/dev/null | wc -l"
Write-Host "  Python files deployed: $fileCount" -ForegroundColor Cyan

$configCheck = ssh "${User}@${RaspiHost}" "grep '$ServerIP' /home/pi/saibatin-asv-main/saibatin_production_fixed.py"
if ($configCheck) {
    Write-Host "  Server IP configured: ✓" -ForegroundColor Green
} else {
    Write-Host "  Server IP configured: ✗ (may need manual check)" -ForegroundColor Yellow
}
Write-Host ""

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "   Deployment Complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "📋 Next Steps:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Start Dashboard Server (di laptop ini):" -ForegroundColor White
Write-Host "   cd c:\Users\Zaky\Downloads\saibatin-asv-main\saibatin-asv-main" -ForegroundColor Cyan
Write-Host "   .\start_dashboard.bat" -ForegroundColor Cyan
Write-Host ""
Write-Host "2. SSH to Raspberry Pi:" -ForegroundColor White
Write-Host "   ssh ${User}@${RaspiHost}" -ForegroundColor Cyan
Write-Host ""
Write-Host "3. Run SAIBATIN system:" -ForegroundColor White
Write-Host "   cd /home/pi/saibatin-asv-main" -ForegroundColor Cyan
Write-Host "   python3 saibatin_production_fixed.py" -ForegroundColor Cyan
Write-Host ""
Write-Host "4. Open Dashboard:" -ForegroundColor White
Write-Host "   http://localhost:5000" -ForegroundColor Cyan
Write-Host ""
Write-Host "5. Test Pixhawk (optional):" -ForegroundColor White
Write-Host "   python3 test_pixhawk.py" -ForegroundColor Cyan
Write-Host ""
