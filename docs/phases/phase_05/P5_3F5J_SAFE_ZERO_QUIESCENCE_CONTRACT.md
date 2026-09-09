# P5-3F5J Safe-Twist Zero-Quiescence Contract

P5-3F5J repairs the integration contract between the generic vehicle-command
Gate and Nav2 Collision Monitor 1.1.20. Collision Monitor may intentionally
stop publishing repeated safe zero commands after `stop_pub_timeout`. The
Gate must not interpret that stationary zero silence as permission to replay a
stale command, nor may it fault solely because a valid stationary zero became
quiet.

## Configuration

`safe_zero_quiescence_enabled` is an explicit opt-in Gate capability.

- Generic/default Gate profiles: `false` (legacy stale-safe behavior retained).
- Phase-5 profile: `true` in `parking_robot_bringup/config/phase5_gate_mock.yaml`.

## Pending arm

With the Phase-5 capability enabled, `arm=true` first evaluates the existing
configuration, publisher authority, authority stability, and validity checks.
If they pass, the service returns success with message `ARM_PENDING_SAFE`.
The public state remains `DISARMED`, `arm_pending=true`, and output remains
exactly zero. The Gate records the monotonic arm-request time.

A safe command received before that time cannot satisfy readiness. A fresh
safe command must have a receipt timestamp strictly later than the arm
request. Until then, safe-message silence alone does not fault. A validity,
authority, or command-integrity failure still fails closed to `FAULT`.

`arm=false` cancels pending readiness and returns
`DISARMED_ARM_PENDING_CANCELLED`.

## Safe zero quiescence

After a genuinely post-arm safe zero is accepted, the Gate may enter the
internal zero-quiescent mode while retaining public state `ARMED`. The
diagnostic reason is `SAFE_ZERO_QUIESCENT`, with `safe_zero_quiescent=true`.
The Gate owns and publishes zero; it never replays the stale safe message.

Zero classification reuses `Twist6.is_zero()` with its existing `1e-9`
absolute tolerance across all six Twist components. A command outside that
tolerance, including a tiny intentional nonzero, is not quiescent.

Zero quiescence is allowed only while all existing independent configuration,
command-integrity, publisher-authority, authority-stability, localization,
controller, and collision-validity checks remain healthy. Any such failure
latches `FAULT` and forces zero.

## Nonzero freshness

If the last accepted post-arm safe command is nonzero, the existing
`safe_twist_timeout_sec` deadman remains unchanged. A stale nonzero command
latches `SAFE_TWIST_STALE` and forces zero.

A fresh safe command after zero quiescence exits quiescence and passes through
the existing validation, limits, and slew logic. If the raw upstream command
becomes nonzero while Collision Monitor is silent, no new safe command arrives
and the Gate remains at zero. If Collision Monitor produces a fresh protected
nonzero command, normal processing resumes.

## Fault reset after zero quiescence

Zero quiescence is separate from the cause of a latched fault. If an
independent collision, localization, controller, authority, configuration, or
command-integrity fault latches `FAULT`, that actual cause must clear before
`arm=false` can reset the Gate. While the cause remains active, reset remains
refused even when the last accepted safe command was zero and stale.

After the independent cause clears, a stale accepted zero alone is not an
active reset-blocking cause in the Phase-5 opt-in profile. Explicit reset
transitions only to `DISARMED` and forces output exactly zero; it never forwards
the stale command. A fault that occurred while `ARM_PENDING_SAFE` can likewise
reset after its actual cause clears without requiring a safe command to have
been accepted.

This exception does not apply to a stale accepted nonzero command. A stale
nonzero remains an active `SAFE_TWIST_STALE` cause until a fresh valid safe
command restores the existing reset condition. After any successful reset,
the next `arm=true` starts a new `ARM_PENDING_SAFE` cycle, and a safe receipt
strictly after that new request is required. A cached zero from the previous
cycle cannot satisfy readiness.

No arm request alone can create motion, and FAULT remains explicitly latched
until its independent cause has cleared and an explicit reset succeeds.
