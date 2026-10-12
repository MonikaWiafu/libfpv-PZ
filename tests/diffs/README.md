# `diff all` corpus

Real `diff all` outputs that pilots posted publicly on the IntoFPV forum
while asking for help. The test suite imports each of them with
`fpv_import_betaflight_diff`, checks that PWM and ARM-on-AUX1 are enforced,
and checks that the result arms and flies.

| File | Firmware | Source |
|---|---|---|
| `bf451-speedybee-f405v4.txt` | Betaflight 4.5.1, STM32F405 | https://intofpv.com/attachment.php?aid=13562 |
| `bf440-speedybee-f405v3.txt` | Betaflight 4.4.0, SPEEDYBEEF405V3 | https://intofpv.com/attachment.php?aid=10199 |
| `bf431-f405.txt` | Betaflight 4.3.1, STM32F405 | https://intofpv.com/attachment.php?aid=9188 |
| `bf430-f405.txt` | Betaflight 4.3.0, STM32F405 | https://intofpv.com/attachment.php?aid=8624 |
| `bf428-tmotorf7.txt` | Betaflight 4.2.8, STM32F7X2 | https://intofpv.com/attachment.php?aid=5735 |
| `bf406-matekf411.txt` | Betaflight 4.0.6, MATEKF411 | https://intofpv.com/attachment.php?aid=3012 |

The files are unmodified. Settings that were renamed since an older
firmware are expected to show up under "Rejected by Betaflight" in the
import report. That is reported to the player, not treated as a failure.

To add a diff, drop a `.txt` file here and add a row to the table.
