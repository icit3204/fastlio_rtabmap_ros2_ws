"""C.4D qualification-only recovery and terminal-drain evidence helpers."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RecoveryInjectionJournal:
    """Online journal validator for canonical-zero, stepwise shadow recovery."""
    rows: list[dict] = field(default_factory=list)
    armed: bool = False
    last_step: int = 0

    def arm(self, *, canonical_count: int, goal_uuid: str) -> None:
        if self.armed or canonical_count != 0 or not goal_uuid:
            raise RuntimeError("RECOVERY_RELAY_ARM_INVALID")
        self.armed = True

    def append(self, row: dict) -> None:
        if not self.armed:
            raise RuntimeError("RECOVERY_RELAY_NOT_ARMED")
        canonical = int(row.get("canonical_recovery_count", -1))
        shadow = int(row.get("actual_shadow_output_count", row.get("shadow_recovery_count", -1)))
        if canonical != 0 or shadow < self.last_step or shadow > 6:
            raise RuntimeError("RECOVERY_CANONICAL_OR_SEQUENCE_INVALID")
        if shadow > self.last_step + 1:
            raise RuntimeError("RECOVERY_STEP_SKIPPED")
        if not row.get("active_goal_uuid") and not row.get("goal_uuid"):
            raise RuntimeError("RECOVERY_UUID_MISSING")
        self.last_step = max(self.last_step, shadow)
        self.rows.append(dict(row))

    def require_sequence(self) -> None:
        observed = {int(r.get("actual_shadow_output_count", r.get("shadow_recovery_count", -1)))
                    for r in self.rows}
        if not all(step in observed for step in range(1, 7)):
            raise RuntimeError("RECOVERY_SEQUENCE_EVIDENCE_INCOMPLETE")


def duration_aware_minimum_publications(elapsed_sec: float, *, rate_hz: float = 9.0,
                                        minimum: int = 5) -> int:
    """Lower bound derived from the real clamp interval, never a fixed tail count."""
    return max(int(minimum), int(float(elapsed_sec) * float(rate_hz)))


@dataclass
class TerminalDrain:
    """Small deterministic state machine preventing cleanup before terminal tail evidence."""
    originating_reason: str
    events: list[str] = field(default_factory=list)
    clamp_joined: bool = False
    physical_closed: bool = False

    def observe(self, event: str) -> None:
        self.events.append(str(event))

    def complete(self) -> bool:
        required = [self.originating_reason, "BLOCK_CANCEL_ACK_ACCEPTED",
                    "CANCELING_3", "CANCELED_5", "BLOCKED",
                    "APPLIED_ZERO_PERSISTENT"]
        positions = []
        for item in required:
            try:
                positions.append(self.events.index(item))
            except ValueError:
                return False
        return positions == sorted(positions) and self.clamp_joined and self.physical_closed


def classify_c4c_attempt1(*, clamp_journal: bool, recovery_steps: set[int],
                          status3: bool, status5: bool, blocked: bool,
                          physical_post_terminal: bool) -> dict:
    """Offline disposition: product branch may be observed without case qualification."""
    branch = bool(recovery_steps and 6 in recovery_steps)
    qualified = (clamp_journal and recovery_steps.issuperset(range(1, 7)) and
                 status3 and status5 and blocked and physical_post_terminal)
    return {"product_branch": "RECOVERY_EXHAUSTED_NO_PROGRESS_OBSERVED" if branch else "NOT_PROVEN",
            "case_qualified": qualified,
            "classification": "CASE_NOT_QUALIFIED" if not qualified else "QUALIFIED_PASS"}
