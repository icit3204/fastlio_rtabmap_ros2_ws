import importlib.util
import os
from pathlib import Path

import pytest


HERE = Path(__file__).parent


def load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


H = load('phase4_p4e6b_timeout_characterization_harness')
S = load('phase4_p4e6b_three_root_selfqual')


@pytest.mark.parametrize('label,states,outcome,stage,complete,cleanup,next_allowed,action', [
    ('A', [], None, 'BEFORE_MATRIX_SPAWN', False, False, True, 'NO_ACTION'),
    ('B', ['MATRIX_SPAWNED'], None, 'MATRIX_SPAWNED_WITNESS_ABSENT', False, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('C', ['MATRIX_SPAWNED', 'WITNESS_SPAWNED'], None, 'MATRIX_AND_WITNESS_SPAWNED_WORKER_ABSENT', False, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('D', ['MATRIX_SPAWNED', 'WITNESS_SPAWNED', 'WORKER_SPAWNED'], None, 'ALL_ROOTS_SPAWNED_BEFORE_HELPER_ENTER', False, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('E', ['MATRIX_SPAWNED', 'WITNESS_SPAWNED', 'WORKER_SPAWNED', 'HELPER_ENTER'], None, 'AFTER_HELPER_ENTER_BEFORE_OUTCOME', False, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('F', ['HELPER_ENTER', 'HELPER_RETURN_READY'], None, 'AFTER_HELPER_RETURN_READY', True, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('G', ['HELPER_ENTER', 'HELPER_EXCEPTION'], 'TIMEOUT', 'AFTER_HELPER_EXCEPTION_OR_TIMEOUT', False, False, False, 'RESUME_PASSIVE_OBSERVATION_IF_SAFE'),
    ('H', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'WORKER_EXITED'], 'TIMEOUT', 'WORKER_EXITED_WITNESS_STILL_OBSERVING', False, False, False, 'RESUME_PASSIVE_OBSERVATION_IF_SAFE'),
    ('I', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_ACTIVE'], 'TIMEOUT', 'POST_HELPER_OBSERVATION_ACTIVE', False, False, False, 'RESUME_PASSIVE_OBSERVATION_IF_SAFE'),
    ('J', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE'], 'TIMEOUT', 'POST_HELPER_OBSERVATION_COMPLETE', True, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('K', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'TERMINATING'], 'TIMEOUT', 'DURING_MATRIX_WITNESS_CLEANUP', True, False, False, 'CLEANUP_ROOTS_AND_TOKEN_DESCENDANTS'),
    ('L', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'CLEANUP_VERIFIED'], 'TIMEOUT', 'CLEANUP_VERIFIED_BUT_CLOSE_MARKER_ABSENT', True, True, False, 'FINALIZE_CHARACTERIZATION_CLOSED'),
    ('M', ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'CLEANUP_VERIFIED', 'CHARACTERIZATION_CLOSED'], 'TIMEOUT', 'CHARACTERIZATION_CLOSED', True, True, True, 'EPISODE_COMPLETE'),
])
def test_reconstructs_all_crash_matrix_states(label, states, outcome, stage, complete, cleanup, next_allowed, action):
    result = H.reconstruct_characterization_episode(states, helper_outcome=outcome)
    assert not result['fatal_inconsistency'], label
    assert result['reconstructed_stage'] == stage
    assert result['observation_complete'] is complete
    assert result['cleanup_verified'] is cleanup
    assert result['next_characterization_allowed'] is next_allowed
    assert result['safe_recovery_action'] == action


@pytest.mark.parametrize('states', [
    ['HELPER_ENTER', 'HELPER_EXCEPTION'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'WORKER_EXITED'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_ACTIVE'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'TERMINATING'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'CLEANUP_VERIFIED'],
    ['HELPER_ENTER', 'HELPER_EXCEPTION', 'POST_HELPER_OBSERVATION_COMPLETE', 'CLEANUP_VERIFIED', 'CHARACTERIZATION_CLOSED'],
])
def test_timeout_is_preserved_through_all_post_outcome_states(states):
    result = H.reconstruct_characterization_episode(states, helper_outcome='TIMEOUT')
    assert result['helper_outcome'] == 'TIMEOUT'
    assert result['helper_success'] is False
    assert 'READINESS_PASS' not in result.values()


def test_actual_ready_is_preserved_but_not_synthesized():
    ready = H.reconstruct_characterization_episode(['HELPER_ENTER', 'HELPER_RETURN_READY', 'CLEANUP_VERIFIED', 'CHARACTERIZATION_CLOSED'])
    assert ready['helper_outcome'] == 'READY' and ready['helper_success']
    passive_only = H.reconstruct_characterization_episode(['HELPER_ENTER', 'POST_HELPER_OBSERVATION_COMPLETE', 'CLEANUP_VERIFIED'])
    assert passive_only['helper_outcome'] is None


@pytest.mark.parametrize('states,alive,reason', [
    (['CHARACTERIZATION_CLOSED'], (), 'CLOSED_WITHOUT_CLEANUP'),
    (['CLEANUP_VERIFIED'], ('witness',), 'CLEANUP_VERIFIED_WITH_LIVE_ROOTS'),
    (['HELPER_ENTER', 'HELPER_RETURN_READY', 'HELPER_EXCEPTION'], (), 'CONFLICTING_HELPER_OUTCOMES'),
    (['POST_HELPER_OBSERVATION_COMPLETE'], (), 'OBSERVATION_COMPLETE_BEFORE_HELPER_ENTER'),
    (['MATRIX_SPAWNED', 'NEW_CHARACTERIZATION_MATRIX_COMMITTED'], (), 'NEW_SAMPLE_BEFORE_PRIOR_CLOSED'),
])
def test_impossible_journals_fail_closed(states, alive, reason):
    result = H.reconstruct_characterization_episode(states, roots_known_alive=alive)
    assert result == {'fatal_inconsistency': True, 'reason': reason}


def test_formal_timeout_still_stops_formal_campaign():
    assert H.formal_terminal('TIMEOUT') == 'READINESS_FAIL'


def test_exact_token_matching_rejects_lookalikes():
    token = 'exact-token'
    assert S.exact_token_match(b'X=1\0P4E6B_EPISODE_TOKEN=exact-token\0', token)
    assert not S.exact_token_match(b'P4E6B_EPISODE_TOKEN=exact-token-extra\0', token)
    assert not S.exact_token_match(b'P4E6B_EPISODE_TOKENISH=exact-token\0', token)
    assert not S.exact_token_match(b'OTHER=prefix-exact-token-suffix\0', token)


def test_parent_does_not_carry_selfqualification_token():
    assert S.TOKEN_KEY not in os.environ


def test_real_non_ros_three_root_escaped_descendant_selfqualification():
    result = S.run_three_root_selfqualification()
    assert result['three_of_three_pass']
    assert result['parent_has_token'] is False
    assert result['token_survivor_term_count'] >= 1
    assert all(row['final_token_count'] == 0 for row in result['scenarios'])
