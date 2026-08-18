"""Qualification-owned atomic Gate-fault injection interlock."""
from dataclasses import dataclass
import json
import os
import time


@dataclass
class InterlockDecision:
    accepted: bool
    forwarded: bool
    reason: str
    request_count: int
    forward_count: int


class InjectionInterlock:
    """Immutable launch-mode gate for qualification fault requests.

    The mode is captured at construction. SHADOW_DENY is the fail-closed
    default. LIVE_SINGLE_SHOT is design/test-only in B2Z and requires the
    exact attempt-scoped authority token.
    """
    def __init__(self, *, mode=None, authority_token=None, evidence_path=None,
                 expected_token=None):
        self.mode = mode if mode in ("SHADOW_DENY", "LIVE_SINGLE_SHOT") else "SHADOW_DENY"
        self.authority_token = authority_token
        self.expected_token = expected_token
        self.evidence_path = evidence_path
        self.request_count = 0
        self.forward_count = 0
        self._latched = False

    def _record(self, item):
        if not self.evidence_path:
            return
        p = os.fspath(self.evidence_path)
        with open(p, "a", encoding="utf-8") as h:
            h.write(json.dumps({"monotonic_ns": time.monotonic_ns(), **item}, sort_keys=True) + "\n")
            h.flush(); os.fsync(h.fileno())

    def request(self, *, parameter, value):
        self.request_count += 1
        if parameter != "force_invalid" or value is not True:
            d = InterlockDecision(False, False, "INVALID_REQUEST", self.request_count, self.forward_count)
        elif self.mode == "SHADOW_DENY":
            self._latched = True
            d = InterlockDecision(False, False, "SHADOW_DENY", self.request_count, self.forward_count)
        elif self.mode != "LIVE_SINGLE_SHOT":
            d = InterlockDecision(False, False, "INVALID_MODE", self.request_count, self.forward_count)
        elif not self.authority_token or self.authority_token != self.expected_token:
            d = InterlockDecision(False, False, "AUTHORITY_DENIED", self.request_count, self.forward_count)
        elif self._latched or self.forward_count:
            d = InterlockDecision(False, False, "SINGLE_SHOT_ALREADY_USED", self.request_count, self.forward_count)
        else:
            self._latched = True
            self.forward_count += 1
            d = InterlockDecision(True, True, "LIVE_SINGLE_SHOT_FORWARD", self.request_count, self.forward_count)
        self._record({"event": "INTERLOCK_DECISION", "parameter": parameter,
                      "value": value, "mode": self.mode, **d.__dict__})
        return d
