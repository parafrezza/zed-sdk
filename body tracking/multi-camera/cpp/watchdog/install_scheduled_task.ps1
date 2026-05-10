param(
    [string]$TaskName = "ZED BodyFusion Watchdog",
    [string]$PythonExe = "python",
    [string]$ConfigPath = "",
    [switch]$AtLogon
)

$ErrorActionPreference = "Stop"

function Resolve-PythonExecutable {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Command
    )

    if (Test-Path $Command) {
        return (Resolve-Path $Command).Path
    }

    $pyLauncher = Join-Path $env:WINDIR "py.exe"
    if (($Command -ieq "python") -or ($Command -ieq "python.exe") -or ($Command -ieq "py") -or ($Command -ieq "py.exe")) {
        if (Test-Path $pyLauncher) {
            return $pyLauncher
        }
    }

    try {
        $resolved = & $Command -c "import sys; print(sys.executable)" 2>$null
        if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($resolved)) {
            $candidate = $resolved.Trim() | Select-Object -First 1
            if (Test-Path $candidate) {
                return (Resolve-Path $candidate).Path
            }
        }
    } catch {
    }

    $commandInfo = Get-Command $Command -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($commandInfo -and $commandInfo.Source -and (Test-Path $commandInfo.Source)) {
        $resolvedPath = (Resolve-Path $commandInfo.Source).Path
        if ($resolvedPath -like "*\WindowsApps\*") {
            throw "Resolved '$Command' to Windows App Execution Alias '$resolvedPath'. This is unreliable for scheduled tasks running as SYSTEM. Pass -PythonExe with a real interpreter path, or install the Python launcher at C:\Windows\py.exe."
        }
        return $resolvedPath
    }

    throw "Unable to resolve Python executable '$Command' to a concrete path. Pass -PythonExe with a full path to python.exe or py.exe."
}

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

$ConfigPath = (Resolve-Path $ConfigPath).Path

$ResolvedPythonExe = Resolve-PythonExecutable -Command $PythonExe

$Arguments = "`"$WatchdogScript`" --config `"$ConfigPath`""
$Action = New-ScheduledTaskAction -Execute $ResolvedPythonExe -Argument $Arguments -WorkingDirectory $ScriptDir

if ($AtLogon) {
    $Trigger = New-ScheduledTaskTrigger -AtLogOn
} else {
    $Trigger = New-ScheduledTaskTrigger -AtStartup
}

if ($AtLogon) {
    $CurrentUser = [System.Security.Principal.WindowsIdentity]::GetCurrent().Name
    $Principal = New-ScheduledTaskPrincipal -UserId $CurrentUser -LogonType Interactive -RunLevel Highest
} else {
    $Principal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -RunLevel Highest
}

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
    -ErrorAction Stop `
    -Force | Out-Null

Write-Host "Installed scheduled task '$TaskName'"
Write-Host "RunAs: $($Principal.UserId)"
Write-Host "Python: $ResolvedPythonExe"
Write-Host "Script: $WatchdogScript"
Write-Host "Config: $ConfigPath"
