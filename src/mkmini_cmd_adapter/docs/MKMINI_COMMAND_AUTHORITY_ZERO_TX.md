# MK-mini command authority and zero-only TX preparation

This document records the K3D commissioning boundary. It is not authorization
to connect a vehicle or transmit on CAN.

## Command profile

The accepted MK-mini command authority is `MKMINI_CTRL_CMD_RESERVED_BITS_36_51_ZERO_AUTHORITY`:

| Field | Wire definition |
|---|---|
| gear | bits 0..3 |
| speed | bits 4..19, unsigned, 0.001 m/s per bit |
| steering | bits 20..35, signed, 0.01 degree per bit; manufacturer inner-wheel angle alpha |
| bits 36..51 | reserved/unassigned for this MK-mini authority; always zero |
| alive | bits 52..55, four-bit rollover |
| checksum | byte 7, XOR of bytes 0..6 |

The generic training-material interpretation of a brake field is not imported.
The current codec has no brake command variable and continues to encode bits
36..51 as zero. Level-4 empirical evidence from the recently used MK-mini
controller also kept these bits zero. This does not authorize nonzero motion.

The labmate bicycle-model beta calculation is historical protocol/chassis
evidence only. `MkminiKinematicsCore` remains the project authority and emits
the manufacturer-defined inner-wheel steering angle alpha.

## Feedback authority split

`feedback_transport_qualified` records K3B static receive/decode capability:
the expected extended IDs, DLC, checksum and alive behavior were observed.
`runtime_feedback_healthy` records the current vehicle state and must be true
separately before a TX authority can qualify. K3B runtime evidence was
unhealthy: reported gear P, mode STOP, vehicle fault level 1, Auto CAN and
Auto IO CAN errors true, and remote-off warning true. Therefore all current
commissioning authority defaults remain false and general physical keyboard TX
remains blocked.

## Stop and stationary candidate

The candidate policy is
`MKMINI_STATIONARY_POLICY_N_ZERO_AFTER_CONFIRMED_STANDSTILL`.

During deceleration, moving D/R is retained in `STOPPING`, with zero speed and
retained steering, until feedback-proven standstill stability. Only then may a
candidate stationary state encode N, zero speed and centered steering. This is
not yet installed-VCU production qualification; P remains the observed
power-on/parking state and is not selected as an ordinary stop command.

## Restricted first-TX session

`ZeroOnlyFirstTxSession` is a separate, authority-gated session. It accepts
only semantic `Gear.N`, speed `0.0`, steering `0.0`, then uses
`MkminiCanCodec` for every frame. It has a nominal 100 Hz caller-ticked
synthesis cadence and increments alive once per emitted frame. The expected
codec-generated vectors are:

* alive 0: `03 00 00 00 00 00 00 03`
* alive 1: `03 00 00 00 00 00 10 13`

`SocketCanTxTransport` is dependency-injectable for fake-socket tests. The
default path is not opened in K3D; `RealTransportAuthority` checks occur before
the transport opens. There is no general keyboard physical-TX enablement.

## Remaining authority

The stationary candidate must be observed and accepted on the installed VCU.
Encoder scaling remains unresolved and is not used for metric odometry. A
future first-TX task must separately authorize hardware, verify no competing
writer, prove current ctrl_fb/Veh_fb_Diag/wheel health, and bound the duration.
