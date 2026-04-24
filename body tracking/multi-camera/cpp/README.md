# ZED Body Fusion C++ Sample

Questo sample ora supporta una configurazione applicativa separata per i parametri ZED e per l'output OSC verso TouchDesigner.

## File di configurazione

Il sample cerca automaticamente `zed_bodyfusion.ini` in questo ordine:

1. cartella dell'eseguibile
2. cartella immediatamente superiore
3. cartella del progetto del sample

Se il file non viene trovato, usa i default compilati nel codice e continua a cercare automaticamente il file `calib_*.json` piu recente.

Puoi anche avviare il sample con:

- un file `.ini` come configurazione applicativa
- un file `.json` come calibrazione ZED360 diretta

## Contratto OSC

L'export OSC usa il namespace:

- `/skeleton/<id>/<standard>/alive`
- `/skeleton/<id>/<standard>/<joint_name>/`

Argomenti:

- `alive`: un intero (`1` presente, `0` scomparso)
- `joint_name`: tre float `x y z` in world space ZED/Fusion

Standard supportati dal sender:

- `zed18`
- `zed34`
- `zed38`

Con la configurazione di default il sample emette `zed18`.

## Esempio rapido

```powershell
cmake -S . -B build
cmake --build build --config Release
.\build\Release\ZED_BodyFusion.exe
```