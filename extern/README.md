# extern/

Fremdcode, der unverändert ins Repo kopiert ist. **Nicht hier bearbeiten.**
Änderungen gehören ins Original-Repo; danach den Ordner komplett durch den
neuen Stand ersetzen und diese Datei aktualisieren.

| Ordner | Quelle | Stand | Zweck |
|---|---|---|---|
| `lisa-if/` | `git@git.tu-berlin.de:rft/homer/lisa-if.git` (`main`) | `69b66a3` „icd upates“, 2026-09-10 | Python-Referenzparser für das UART-Protokoll des Roboterarms (LISA-ODISEE ICD v0.9). Gegenstück in der Firmware: `include/hal/arm_hal.hpp` |

PlatformIO baut nur `src/`, daher landet nichts von hier in der Firmware.
