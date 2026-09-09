# MK-mini receive-only capture contract

This is preparation for a later, separately authorized installed-unit
qualification. It does not open the current host interface, transmit, or
enable motion.

## Installed-unit identity

`config/mkmini_installed_unit_identity.yaml` is a machine-readable worksheet.
Allowed states are `VALUE`, `NOT_VISIBLE`, `NOT_PRESENT`, and `UNKNOWN`. The
current exterior inspection found no usable labels, so the relevant fields are
recorded as `NOT_VISIBLE`; no values are fabricated. The operator should
transcribe only visible labels for the VCU/controller, rear drive, and
encoder/gearbox. Removing or loosening hardware is not authorized.

The supplied manuals, vendor ROS material, and generic training material do not
provide a proven mapping from a label/version field to either the disputed
brake layout or the competing encoder scale. Therefore:

```text
LABEL_IDENTITY_NOT_SUFFICIENT_TO_RESOLVE_PROFILE
```

Identity capture is still necessary evidence, but it is not by itself a brake
or encoder authority claim.

## Receive-only transport

`SocketCanReadOnlyTransport` has only `open()`, `receive()`, and `close()`.
The socket factory and clock are injectable. Tests use a fake socket. The
default receive filter contains only the known extended feedback IDs:

```text
0x18C4D2EF  ctrl_fb
0x18C4D7EF  rear-left wheel
0x18C4D8EF  rear-right wheel
0x18C4EAEF  Veh_fb_Diag
0x18C4E1EF  BMS information
0x18C4E2EF  BMS flags
0x18C4DEEF  odometer
0x18C4E8EF  ultrasonic 1
0x18C4E9EF  ultrasonic 2
```

The command ID is not a default receive filter or an output path. For a
separate, explicitly passive observation, a caller may supply a command-ID
receive filter to the same receive-only class; that path still has no command
synthesis or output method. Known feedback is decoded by the established
`MkminiFeedbackCodec`; unknown IDs, raw payloads,
timestamps, and decode failures remain in the capture record. A bad checksum
is retained as invalid evidence rather than silently discarded.

For an unknown-ID discovery/archive pass, the caller supplies `filters=()`;
this explicitly disables kernel filtering and preserves every received frame.

`CaptureAccumulator` reports per-ID count, rate, payload variation, alive
values, and checksum counts. `JsonlCaptureArchive` preserves raw payload and
metadata for offline reinterpretation.

## What read-only capture can establish

Potentially observable: feedback IDs, extended/standard flag, DLC, rates,
payload variability, current mode, reported gear, diagnostic/fault fields,
checksum validity, alive progression, and wheel pulse/sign activity.

Not guaranteed from idle capture alone: installed brake command semantics,
stationary gear behavior, pulses-per-wheel-revolution, gearbox ratio, or the
meaning of a disputed command field. Encoder scale still requires an
independent authority or controlled wheel/count experiment. Brake profile
remains unresolved until a profile-specific authority is obtained.

## Later hardware procedure

After an explicit supervisor authorization: keep the robot stationary, stop
and verify every command writer, connect CAN, open this receive-only capture,
record 30–60 seconds of idle traffic, optionally perform only separately
authorized native-control observations, close the capture, disconnect CAN, and
analyze the archive offline. No Jetson command writer is part of this path.
