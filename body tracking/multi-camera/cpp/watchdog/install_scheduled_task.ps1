param(
    [string]$TaskName = "ZED BodyFusion Watchdog",
    [string]$PythonExe = "python",
    [string]$ConfigPath = "",
    [switch]$AtLogon
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$WatchdogScript = Join-Path $ScriptDir "zed_bodyfusion_watchdog.py"

if (-not (Test-Path $WatchdogScript)) {
    throw "Watchdog script not found: $WatchdogScript"
}

if ([string]::IsNullOrWhiteSpace($ConfigPath)) {
    $Candidate = Join-Path $ScriptDir "watchdog_config.json"
    if (Test-Path $Candidate) {
        $ConfigPath = $Candidate
    } else {
        $ConfigPath = Join-Path $ScriptDir "watchdog_config.example.json"
    }
}

$Arguments = "`"$WatchdogScript`" --config `"$ConfigPath`""
$Action = New-ScheduledTaskAction -Execute $PythonExe -Argument $Arguments -WorkingDirectory $ScriptDir

if ($AtLogon) {
    $Trigger = New-ScheduledTaskTrigger -AtLogOn
} else {
    $Trigger = New-ScheduledTaskTrigger -AtStartup
}

$Principal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -RunLevel Highest
$Settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries `
    -ExecutionTimeLimit (New-TimeSpan -Days 0) `
    -RestartCount 999 `
    -RestartInterval (New-TimeSpan -Minutes 1)

Register-ScheduledTask `
    -TaskName $TaskName `
    -Action $Action `
    -Trigger $Trigger `
    -Principal $Principal `
    -Settings $Settings `
    -Force | Out-Null

Write-Host "Installed scheduled task '$TaskName'"
Write-Host "Script: $WatchdogScript"
Write-Host "Config: $ConfigPath"
