# Return to base by watering-event count and a payload-matched QR

The robot ends a run by driving to a fixed base once it has performed
`PLANTS_PER_RUN` **watering events** (pump activations), not once it has watered
that many *distinct* plants. We chose watering-event counting over the existing
ReID-distinct count (`len(_watered_ids)`) because it is predictable for the
operator ("set 2, the pump runs twice, then it goes home") and does not depend on
ReID/track-ID accuracy.

The base is identified by a QR code whose decoded payload must equal
`BASE_QR_PAYLOAD`, decoded with OpenCV's built-in `cv2.QRCodeDetector` (no new
dependency). We match the payload rather than treating any QR as the base so a
stray QR in the field can't be mistaken for home. Detection runs inside the
existing inference loop only while *returning*, reusing the plant
zone/center/approach + area-ratio "arrived" logic to drive toward it. If the base
isn't in view the robot scans indefinitely (no timeout) — chosen for simplicity;
revisit if field runs show it spinning forever is a real failure mode.
