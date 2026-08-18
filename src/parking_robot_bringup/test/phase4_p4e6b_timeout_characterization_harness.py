"""Offline-only outcome-neutral timeout characterization transaction model."""
from dataclasses import dataclass


FORMAL_TERMINALS = {"READINESS_PASS", "READINESS_FAIL"}
CHARACTERIZATION_TERMINAL = "CHARACTERIZATION_CLOSED"


@dataclass(frozen=True)
class CharacterizationOutcome:
    helper_outcome: str  # READY, TIMEOUT, OTHER_EXCEPTION
    helper_success: bool
    characterization_closed: bool


def formal_terminal(outcome):
    return "READINESS_PASS" if outcome == "READY" else "READINESS_FAIL"


def characterization_terminal(outcome, observation_complete, cleanup_verified):
    return CharacterizationOutcome(outcome, outcome == "READY",
                                  bool(observation_complete and cleanup_verified))


def may_start_next_characterization(previous):
    return bool(previous and previous.characterization_closed)


def audit_admission(rows):
    """Every matrix commit must have the immediately preceding same-identity admission."""
    for index, row in enumerate(rows):
        if row.get("state") != "MATRIX_COMMITTED":
            continue
        if index == 0:
            return False
        prior = rows[index - 1]
        if prior.get("state") != "PRE_EPISODE_ADMISSION_PASS":
            return False
        if any(prior.get(k) != row.get(k) for k in ("episode_index", "episode_uuid", "token", "domain")):
            return False
    return True


def reconstruction_state(states):
    """Crash recovery: incomplete observation is never eligible for continuation."""
    if "POST_HELPER_OBSERVATION_ACTIVE" in states and "POST_HELPER_OBSERVATION_COMPLETE" not in states:
        return "BLOCKED_OBSERVATION_INCOMPLETE"
    if "CHARACTERIZATION_CLOSED" in states:
        return "CLOSED"
    return "BLOCKED_UNFINISHED"

def three_root_state(matrix_alive, worker_alive, witness_alive, observation_complete):
    return {'matrix_alive':matrix_alive,'worker_alive':worker_alive,'witness_alive':witness_alive,
            'cleanup_required': matrix_alive or worker_alive or witness_alive,
            'next_allowed': observation_complete and not (matrix_alive or worker_alive or witness_alive)}

@dataclass
class Root:
    pid:int; pgid:int; alive:bool=True; spawn_monotonic_ns:int=0; exit_monotonic_ns:int|None=None
@dataclass
class ThreeRootEpisode:
    token:str; episode_uuid:str; domain:int; matrix:Root; worker:Root; witness:Root; helper_outcome:str|None=None; observation_complete:bool=False; cleanup_verified:bool=False
    def worker_exited(self,when): self.worker.alive=False; self.worker.exit_monotonic_ns=when
    def closeable(self): return self.helper_outcome is not None and self.observation_complete and self.cleanup_verified

def reconstruct_characterization(states, helper_outcome=None):
    complete='POST_HELPER_OBSERVATION_COMPLETE' in states; cleaned='CLEANUP_VERIFIED' in states; closed='CHARACTERIZATION_CLOSED' in states
    if closed:return {'next_allowed':True,'cleanup_required':False,'observation_complete':True,'helper_outcome':helper_outcome,'action':'NONE'}
    if helper_outcome and complete and cleaned:return {'next_allowed':False,'cleanup_required':False,'observation_complete':True,'helper_outcome':helper_outcome,'action':'FINALIZE_CHARACTERIZATION_CLOSED'}
    return {'next_allowed':False,'cleanup_required':not cleaned,'observation_complete':complete,'helper_outcome':helper_outcome,'action':'RECOVER_AND_BLOCK'}

def reconstruct_characterization_episode(states, roots_known_alive=(), helper_outcome=None):
    """Reconstruct one A–M characterization episode from durable evidence only.

    ``roots_known_alive`` is supplied by durable root-exit/token-scan evidence;
    absence of an exit record is represented as a *possibly* active root.  This
    deliberately never relaunches a helper and never derives helper READY from
    passive observations.
    """
    s = set(states)
    closed = 'CHARACTERIZATION_CLOSED' in s
    cleanup = 'CLEANUP_VERIFIED' in s
    explicit_alive = tuple(roots_known_alive)

    fatal = None
    if closed and not cleanup:
        fatal = 'CLOSED_WITHOUT_CLEANUP'
    elif cleanup and explicit_alive:
        fatal = 'CLEANUP_VERIFIED_WITH_LIVE_ROOTS'
    elif 'HELPER_RETURN_READY' in s and 'HELPER_EXCEPTION' in s:
        fatal = 'CONFLICTING_HELPER_OUTCOMES'
    elif 'POST_HELPER_OBSERVATION_COMPLETE' in s and 'HELPER_ENTER' not in s:
        fatal = 'OBSERVATION_COMPLETE_BEFORE_HELPER_ENTER'
    elif 'NEW_CHARACTERIZATION_MATRIX_COMMITTED' in s and not closed:
        fatal = 'NEW_SAMPLE_BEFORE_PRIOR_CLOSED'
    if fatal:
        return {'fatal_inconsistency': True, 'reason': fatal}

    outcome = helper_outcome or (
        'READY' if 'HELPER_RETURN_READY' in s else
        ('TIMEOUT' if 'HELPER_EXCEPTION' in s else None))
    # READY requires only its normal flush; timeout/other exception requires
    # explicit post-helper observation completion.
    observation_complete = ('POST_HELPER_OBSERVATION_COMPLETE' in s or
                            outcome == 'READY')

    if closed:
        stage, implied = 'CHARACTERIZATION_CLOSED', ()
    elif cleanup:
        stage, implied = 'CLEANUP_VERIFIED_BUT_CLOSE_MARKER_ABSENT', ()
    elif 'TERMINATING' in s:
        stage, implied = 'DURING_MATRIX_WITNESS_CLEANUP', ('matrix', 'witness')
    elif 'POST_HELPER_OBSERVATION_COMPLETE' in s:
        stage, implied = 'POST_HELPER_OBSERVATION_COMPLETE', ('matrix', 'witness')
    elif 'POST_HELPER_OBSERVATION_ACTIVE' in s:
        stage, implied = 'POST_HELPER_OBSERVATION_ACTIVE', ('matrix', 'witness')
    elif 'WORKER_EXITED' in s:
        stage, implied = 'WORKER_EXITED_WITNESS_STILL_OBSERVING', ('matrix', 'witness')
    elif outcome:
        stage, implied = 'AFTER_HELPER_EXCEPTION_OR_TIMEOUT' if outcome != 'READY' else 'AFTER_HELPER_RETURN_READY', ('matrix', 'worker', 'witness')
    elif 'HELPER_ENTER' in s:
        stage, implied = 'AFTER_HELPER_ENTER_BEFORE_OUTCOME', ('matrix', 'worker', 'witness')
    elif 'WORKER_SPAWNED' in s:
        stage, implied = 'ALL_ROOTS_SPAWNED_BEFORE_HELPER_ENTER', ('matrix', 'worker', 'witness')
    elif 'WITNESS_SPAWNED' in s:
        stage, implied = 'MATRIX_AND_WITNESS_SPAWNED_WORKER_ABSENT', ('matrix', 'witness')
    elif 'MATRIX_SPAWNED' in s:
        stage, implied = 'MATRIX_SPAWNED_WITNESS_ABSENT', ('matrix',)
    else:
        stage, implied = 'BEFORE_MATRIX_SPAWN', ()

    possible_roots = explicit_alive or implied
    cleanup_required = (not cleanup and (bool(possible_roots) or 'MATRIX_SPAWNED' in s))
    if closed:
        action = 'EPISODE_COMPLETE'
    elif cleanup and observation_complete and outcome is not None:
        action = 'FINALIZE_CHARACTERIZATION_CLOSED'
    elif outcome is not None and not observation_complete:
        action = 'RESUME_PASSIVE_OBSERVATION_IF_SAFE'
    elif cleanup_required:
        action = 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'
    else:
        action = 'NO_ACTION'
    return {
        'fatal_inconsistency': False,
        'reconstructed_stage': stage,
        'helper_outcome': outcome,
        'helper_outcome_known': outcome is not None,
        'observation_complete': observation_complete,
        'cleanup_verified': cleanup,
        'cleanup_required': cleanup_required,
        'active_or_possibly_active_roots': possible_roots,
        'characterization_closed': closed,
        # Before any matrix commit no system episode exists, so a fresh sample
        # is permitted.  After commit, only the immutable closure permits one.
        'next_characterization_allowed': (not ('MATRIX_SPAWNED' in s or 'HELPER_ENTER' in s or outcome is not None)) or (closed and cleanup),
        'safe_recovery_action': action,
        'helper_success': outcome == 'READY',
    }
