# Runtime settings as a config-default + override layer, applied on Start

`config.py` stays the single source of **default** values for every tunable
(including the navigator constants, which were moved here from `automatic.py`).
`components/settings.py` adds a thin override layer persisted to
`Backend/runtime_settings.json`, edited via a small UI at `/settings/ui`. We
chose this over rewriting `config.py` from the UI (fragile, needs a process
restart, risks corrupting source) and over a full database/settings framework
(overkill for ~30 knobs on a Pi).

Run-scope (behaviour) settings are applied by **re-binding the module-level
globals** in `automatic` and `ultrasonic` from the effective settings at
`AutoNavigator.start()` — not by reading a settings object on every loop
iteration. This is the surprising bit: it means edits take effect on the next
**Start** (stable snapshot per run, no per-cycle lookups, zero churn in the
tuned navigation loop), and that the navigation code keeps using plain module
constants. Hardware/wiring settings are deliberately **read-only** in the UI:
they are consumed once at boot and a wrong GPIO pin entered in a web form would
silently disable the motors, so changing them stays a `config.py` + restart
operation.
