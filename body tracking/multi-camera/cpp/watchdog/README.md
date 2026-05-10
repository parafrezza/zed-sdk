# ZED BodyFusion Watchdog

Supervisor esterno per `ZED_BodyFusion.exe`. Non modifica l'app C++: la avvia, la rilancia se termina in modo inatteso, espone comandi TCP/UDP e deduce lo stato camere dai log gia prodotti dall'app.

## Configurazione

1. Compila l'app C++:

```powershell
cmake -S .. -B ..\build
cmake --build ..\build --config Release
```

2. Copia l'esempio e adatta i path:

```powershell
Copy-Item .\watchdog_config.example.json .\watchdog_config.json
notepad .\watchdog_config.json
```

Valori principali:

- `app_executable`: path di `ZED_BodyFusion.exe`
- `app_args`: opzionale, ad esempio `["../zed_bodyfusion.ini"]` o `["../calib_00.json"]`
- `app_working_dir`: directory di lavoro dell'app
- `bind_host`: usa `127.0.0.1` se Companion gira sullo stesso PC; usa `0.0.0.0` solo se devi controllarlo da rete
- `auth_token`: consigliato se `bind_host` non e localhost

## Avvio manuale

```powershell
python .\zed_bodyfusion_watchdog.py --config .\watchdog_config.json
```

Verifica solo config:

```powershell
python .\zed_bodyfusion_watchdog.py --config .\watchdog_config.json --check-config
```

## API TCP/UDP

Default: TCP `127.0.0.1:8765`, UDP `127.0.0.1:8765`.

Comandi testuali:

- `health`
- `ping`
- `start`
- `stop`
- `restart`
- `sleep`
- `shutdown`
- `reboot`
- `wake`

`restart` riavvia l'app. Per riavviare Windows usa `reboot`, `system_restart` o `pc_restart`.

Risposta: JSON su una riga. Esempio TCP:

```powershell
"health`n" | nc 127.0.0.1 8765
```

Esempio risposta:

```json
{"ok":true,"status":"ok","app":"running","pid":1234,"expected_cameras":2,"subscribed_cameras":2}
```

Se imposti `auth_token`, i comandi mutanti vanno inviati cosi:

```text
token=SEGRETO restart
```

oppure in JSON:

```json
{"command":"restart","token":"SEGRETO"}
```

`health` resta pubblico di default; cambia `auth_required_for_health` a `true` se serve.

## Installazione all'avvio di Windows

Apri PowerShell come amministratore nella cartella `watchdog`:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\install_scheduled_task.ps1 -ConfigPath "C:\path\to\watchdog_config.json"
```

Il task viene registrato come `SYSTEM` con `RunLevel Highest`, quindi i comandi `sleep`, `shutdown` e `reboot` sono disponibili se `require_elevation_for_system_commands` e `true`.

L'installer ora prova a risolvere automaticamente un path assoluto per Python. Evita alias come `python` o `py` se puntano a `WindowsApps`, perche un task eseguito come `SYSTEM` spesso fallisce con `0x80070002`.

Se vuoi forzare un interprete specifico, ricava prima il path reale con `python -c "import sys; print(sys.executable)"` e passalo a `-PythonExe`.

Per installarlo all'accesso utente invece che al boot:

```powershell
.\install_scheduled_task.ps1 -AtLogon
```

Con `-AtLogon` il task viene registrato per l'utente corrente con logon interattivo. Questo e il modo corretto se vuoi vedere la finestra di `ZED_BodyFusion.exe` sul desktop.

Senza `-AtLogon`, il task gira come `SYSTEM` all'avvio di Windows. Va bene per un servizio headless controllato via watchdog, ma una GUI lanciata cosi resta nella Session 0 e non e visibile sul desktop dell'utente.

## Verifica dopo installazione o riavvio

Apri PowerShell come amministratore e verifica che il task esista e abbia tentato l'avvio:

```powershell
Get-ScheduledTask -TaskName "ZED BodyFusion Watchdog" |
	Select-Object TaskName, State, TaskPath

Get-ScheduledTaskInfo -TaskName "ZED BodyFusion Watchdog" |
	Select-Object LastRunTime, LastTaskResult, NextRunTime
```

In alternativa puoi usare anche `schtasks`:

```powershell
schtasks /Query /TN "ZED BodyFusion Watchdog" /V /FO LIST
```

Se il task e registrato, controlla che il watchdog stia ascoltando sulla porta configurata e che risponda al probe di health:

```powershell
Test-NetConnection 127.0.0.1 -Port 8765
Invoke-RestMethod -Uri "http://127.0.0.1:8765/health" | ConvertTo-Json -Depth 4
```

Controllo rapido dei processi e dei log:

```powershell
Get-Process python, py, ZED_BodyFusion -ErrorAction SilentlyContinue |
	Select-Object Id, ProcessName, StartTime, Path

Get-ChildItem .\logs |
	Sort-Object LastWriteTime -Descending |
	Select-Object -First 5 Name, LastWriteTime, Length

Get-Content (
	(Get-ChildItem .\logs | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
) -Tail 80
```

Interpretazione rapida:

- `State` puo essere `Ready` o `Running`; `LastTaskResult` dovrebbe essere `0`.
- `2147942402` o `-2147024894` corrispondono a `0x80070002`: il task non trova l'eseguibile configurato, tipicamente `python` registrato come nome generico invece che come path assoluto.
- `Test-NetConnection` deve riportare `TcpTestSucceeded : True`.
- `/health` deve restituire JSON con `"ok": true`.
- Se il task esiste ma `/health` non risponde, guarda l'ultimo file in `logs` e la cronologia del task.

Per forzare una prova manuale senza reinstallare:

```powershell
Start-ScheduledTask -TaskName "ZED BodyFusion Watchdog"
```

Se vedi `0x80070002`, reinstalla il task in modo che salvi il path assoluto del Python interpreter:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\install_scheduled_task.ps1 -ConfigPath ".\watchdog_config.json"
```

Poi verifica che `Task To Run` in `schtasks` non inizi con `python`, ma con un percorso completo tipo `C:\...\python.exe`.

Se `health` dice `app: running` ma la finestra non si vede, controlla la sessione del processo:

```powershell
Get-CimInstance Win32_Process -Filter "Name = 'ZED_BodyFusion.exe'" |
	Select-Object ProcessId, Name, SessionId
```

Se `SessionId` e `0`, l'app e stata lanciata come servizio in Session 0. Per renderla visibile, reinstalla il task in modalita logon utente:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\install_scheduled_task.ps1 -AtLogon -ConfigPath ".\watchdog_config.json"
```

Poi esci e rientra nella sessione utente, oppure avvia il task manualmente dopo la reinstallazione.

## Wake

Un PC gia in sleep non puo rispondere a un endpoint locale, perche il watchdog non sta girando. Il comando `wake` invia pacchetti Wake-on-LAN ai MAC configurati in `wake_macs`; va chiamato da un'altra macchina o da un media server ancora acceso.

## Note operative

- Se l'app esce da sola e `auto_restart` e `true`, viene rilanciata dopo `restart_delay_seconds`.
- Se invii `stop`, il watchdog non la rilancia finche non riceve `start` o `restart`.
- I log dell'app sono salvati in `log_dir`.
- Il conteggio `subscribed_cameras` e stimato dai log `ZED ... subscribed`, `All N configured cameras` e `Running in degraded mode with X/Y configured cameras`.
