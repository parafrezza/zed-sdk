# ZED Body Fusion C++ Sample

Questo sample ora supporta una configurazione applicativa separata per i parametri ZED e per l'output OSC verso TouchDesigner.

## Configurazione aggiuntiva

Sezione `fusion`:

- `body_format = 18|34|38` definisce il formato OSC atteso. Se la Fusion restituisce un numero di keypoint diverso, il sender si riallinea automaticamente al formato reale (`zed18`, `zed34`, `zed38`).

Nota sui due `body_format` nel file INI:

- `publisher.body_format` controlla il formato stimato da ogni camera prima della Fusion
- `fusion.body_format` resta il formato di riferimento lato sample per l'export fuso
- `osc.output_standard` decide il contratto OSC finale: automatico oppure forzato con remap verso `zed18`, `zed34` o `zed38`

Sezione `preview`:

- `enabled = 0|1` abilita o disabilita completamente la preview OpenGL e il recupero di immagini/point cloud per risparmiare risorse.

Sezione `osc`:

- `output_standard = auto|zed18|zed34|zed38` controlla il formato OSC emesso. `auto` segue il formato reale della Fusion; un valore fisso prova a rimappare i joint verso quello standard prima dell'invio.
- `log_messages = 0|1` abilita il log dei messaggi OSC inviati
- `log_file = <path>` definisce il file di log; se vuoto usa `zed_bodyfusion_osc.log` accanto all'eseguibile

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

Con `BODY_18` e `osc.output_standard = zed18`, ogni body produce sempre 19 messaggi/entry logiche: 18 joint e 1 `alive`.

Quando un body scompare, il sender riemette l'ultimo set di joint noto per quell'ID insieme a `alive = 0`, cosi il frame di uscita resta coerente con lo stesso conteggio di 19.

Standard supportati dal sender:

- `zed18`
- `zed34`
- `zed38`

Politica di output:

- `auto`: emette il formato reale disponibile (`zed18`, `zed34`, `zed38`)
- `zed18`: se la Fusion produce 34 o 38 joint, il sender invia solo il sottoinsieme compatibile a 18 joint
- `zed34`: se la Fusion produce 38 joint, il sender invia il sottoinsieme compatibile a 34 joint
- `zed38`: richiede una sorgente gia compatibile a 38 joint

Se il mapping richiesto non e possibile, il body viene scartato e viene scritto un warning nel log verbose.

Quando il remap avviene davvero, il sample scrive anche un log esplicito in console, ad esempio `Remapping fused body from zed34 to zed18`.

Con la configurazione di default il sample usa `output_standard = auto`.

## Esempio rapido

```powershell
cmake -S . -B build
cmake --build build --config Release
.\build\Release\ZED_BodyFusion.exe
```

## Mappatura zed18

- `0`: `nose`
- `1`: `neck`
- `2`: `right_shoulder`
- `3`: `right_elbow`
- `4`: `right_wrist`
- `5`: `left_shoulder`
- `6`: `left_elbow`
- `7`: `left_wrist`
- `8`: `right_hip`
- `9`: `right_knee`
- `10`: `right_ankle`
- `11`: `left_hip`
- `12`: `left_knee`
- `13`: `left_ankle`
- `14`: `right_eye`
- `15`: `left_eye`
- `16`: `right_ear`
- `17`: `left_ear`