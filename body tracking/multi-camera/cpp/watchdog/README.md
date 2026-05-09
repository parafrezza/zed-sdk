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
.\install_scheduled_task.ps1 -PythonExe "C:\Windows\py.exe" -ConfigPath "C:\path\to\watchdog_config.json"
```

Il task viene registrato come `SYSTEM` con `RunLevel Highest`, quindi i comandi `sleep`, `shutdown` e `reboot` sono disponibili se `require_elevation_for_system_commands` e `true`.

Per installarlo all'accesso utente invece che al boot:

```powershell
.\install_scheduled_task.ps1 -AtLogon
```

## Wake

Un PC gia in sleep non puo rispondere a un endpoint locale, perche il watchdog non sta girando. Il comando `wake` invia pacchetti Wake-on-LAN ai MAC configurati in `wake_macs`; va chiamato da un'altra macchina o da un media server ancora acceso.

## Note operative

- Se l'app esce da sola e `auto_restart` e `true`, viene rilanciata dopo `restart_delay_seconds`.
- Se invii `stop`, il watchdog non la rilancia finche non riceve `start` o `restart`.
- I log dell'app sono salvati in `log_dir`.
- Il conteggio `subscribed_cameras` e stimato dai log `ZED ... subscribed`, `All N configured cameras` e `Running in degraded mode with X/Y configured cameras`.
