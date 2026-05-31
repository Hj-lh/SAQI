# SAQI — Autonomous Watering Robot

A Raspberry Pi differential-drive robot that finds plants with a YOLO camera,
waters them, and (this iteration) returns to a fixed home base when its watering
quota for the run is met. This glossary fixes the vocabulary the navigation code
and config share.

## Language

**Run**:
A single autonomous session — from starting auto mode until the robot ends the
mission at the base (or is stopped manually). Mission state resets at the start
of each run.
_Avoid_: session, cycle.

**Watering event**:
One completed activation of the pump on a plant. The mission counts watering
events, **not** distinct plants — watering the same plant twice counts twice.
_Avoid_: plant watered, watering cycle.

**Plants per run**:
The number of watering events after which the robot stops hunting for plants and
returns to base (`PLANTS_PER_RUN`). `0` means unlimited — it never returns.

**Base**:
The fixed home position the robot drives to once its watering quota is met,
marked by a QR code. Reaching the base ends the run.
_Avoid_: home, dock, station.

**Base QR**:
The QR code that marks the base. A QR is only treated as the base when its
decoded payload matches `BASE_QR_PAYLOAD` exactly — other QR codes in view are
ignored.

**Returning**:
The navigator phase entered after the watering quota is met. In this phase the
robot looks only for the Base QR (not plants) and drives toward it, scanning
indefinitely if it isn't in view.

**Arrived (at base)**:
The Base QR's bounding box is centered and fills at least
`BASE_ARRIVAL_AREA_RATIO` of the frame. Triggers mission completion: motors
stop, auto mode turns off, LEDs return to idle.

**Setting**:
A tunable value with a default in `config.py`, optionally overridden at runtime
via the settings UI. Each setting has a **scope**.

**Run-scope setting**:
A behaviour/tuning value read by the navigator during a run (speeds, durations,
thresholds, mission knobs, obstacle avoidance). Editable in the UI; an override
is applied when the next auto **run** starts ("apply on Start").
_Avoid_: live setting.

**Read-only setting**:
A hardware/wiring value read once when the backend boots (GPIO pins, PWM freq,
pump pin/polarity, LED, serial port, camera URL). Shown in the UI for reference;
changing it means editing `config.py` and restarting.

## Example dialogue

> **Dev:** If `PLANTS_PER_RUN` is 2 and the robot waters one plant, loses it,
> then re-finds and waters it again, does it head home?
>
> **Domain expert:** Yes. We count *watering events*, not distinct plants. Two
> pump activations = quota met = start *returning*, regardless of whether ReID
> thinks it's the same plant.
>
> **Dev:** And if it sees a QR on a fertilizer bag while returning?
>
> **Domain expert:** Ignored. Only a QR whose payload equals `BASE_QR_PAYLOAD`
> counts as the *base*. Anything else isn't the base.
