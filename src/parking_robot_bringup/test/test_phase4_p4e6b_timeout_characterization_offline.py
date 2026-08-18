import importlib.util
from pathlib import Path
import pytest
import json

D=Path(__file__).parent
def load(name):
 s=importlib.util.spec_from_file_location(name,D/f'{name}.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
H=load('phase4_p4e6b_timeout_characterization_harness'); A=load('phase4_p4e6b_passive_eligibility_analyzer'); W=load('phase4_p4e6b_passive_readiness_witness')

def sample(t=4, **x):
 r={'monotonic_ns':int(t*10**9),'publisher_count':1,'publisher_node':'collision_monitor_validity_monitor','publisher_gid':'g','diagnostic_gid':'d','post_epoch_bool_count':2,'bool_value':True,'bool_age_ns':1,'diagnostic_state':'VALID','diagnostic_reason':'VALID','healthy_stable_sec':1.,'diagnostic_age_ns':1,'diagnostic_message_age_ns':1,'diagnostic_header_ros_ns':2,'epoch_ros_ns':1,'source_age_upper_bound_sec':.1};r.update(x);return r

def test_formal_timeout_is_failure(): assert H.formal_terminal('TIMEOUT')=='READINESS_FAIL'
def test_characterization_timeout_is_not_success(): assert H.characterization_terminal('TIMEOUT',True,True)==H.CharacterizationOutcome('TIMEOUT',False,True)
def test_next_allowed_only_when_closed(): assert H.may_start_next_characterization(H.characterization_terminal('TIMEOUT',True,True)); assert not H.may_start_next_characterization(H.characterization_terminal('TIMEOUT',False,True))
def test_incomplete_observation_blocks_recovery(): assert H.reconstruction_state(['POST_HELPER_OBSERVATION_ACTIVE'])=='BLOCKED_OBSERVATION_INCOMPLETE'
def test_closed_recovery(): assert H.reconstruction_state(['CHARACTERIZATION_CLOSED'])=='CLOSED'
def test_admission_auditor_accepts_exact_pair(): assert H.audit_admission([{'state':'PRE_EPISODE_ADMISSION_PASS','episode_index':1,'episode_uuid':'u','token':'t','domain':1},{'state':'MATRIX_COMMITTED','episode_index':1,'episode_uuid':'u','token':'t','domain':1}])
@pytest.mark.parametrize('bad',[{}, {'state':'PLANNED','episode_index':1,'episode_uuid':'u','token':'t','domain':1}])
def test_admission_auditor_rejects_missing_or_wrong(bad): assert not H.audit_admission([bad,{'state':'MATRIX_COMMITTED','episode_index':1,'episode_uuid':'u','token':'t','domain':1}])
@pytest.mark.parametrize('field,value',[('bool_value',False),('diagnostic_state','INVALID'),('diagnostic_reason','OTHER'),('healthy_stable_sec',.9),('post_epoch_bool_count',1),('bool_age_ns',250_000_000),('diagnostic_age_ns',250_000_000),('diagnostic_message_age_ns',250_000_000),('source_age_upper_bound_sec',.5),('publisher_count',2)])
def test_each_predicate_blocks(field,value): assert A.first_passive_ready_eligible([sample(**{field:value})],0)['event']=='NONE_BY_BOUND'
def test_eligible_time(): assert A.first_passive_ready_eligible([sample(14)],0)['monotonic_ns']==14*10**9
def test_bound(): assert A.first_passive_ready_eligible([sample(21)],0)['event']=='NONE_BY_BOUND'
def test_pre_helper_eligibility_is_not_selected():
    assert A.first_passive_ready_eligible([sample(4)],5*10**9)['event']=='NONE_BY_BOUND'
def test_pre_and_post_helper_selects_post_helper():
    r=A.first_passive_ready_eligible([sample(4),sample(6)],5*10**9)
    assert r['monotonic_ns']==6*10**9 and r['analysis_window_start_ns']==5*10**9
def test_exact_helper_lower_bound_is_inclusive():
    r=A.first_passive_ready_eligible([sample(5)],5*10**9)
    assert r['monotonic_ns']==5*10**9
@pytest.mark.parametrize('t,expected',[ (7.999999999,'READY_WITHIN_8S'), (8.0,'READY_AFTER_8S') ])
def test_eight_second_classification(t,expected):
    r=A.analyze_passive_eligibility([sample(t)],0,helper_outcome='READY')
    assert r['eligibility_class']==expected
def test_timeout_eligibility_classification():
    assert A.analyze_passive_eligibility([sample(7)],0,helper_outcome='TIMEOUT')['eligibility_class']=='ELIGIBLE_BEFORE_8S'
    assert A.analyze_passive_eligibility([sample(8)],0,helper_outcome='TIMEOUT')['eligibility_class']=='ELIGIBLE_AT_OR_AFTER_8S'
@pytest.mark.parametrize('bad',[None,True,-1,1.5])
def test_missing_or_malformed_helper_enter_rejected(bad):
    with pytest.raises(ValueError): A.first_passive_ready_eligible([sample(1)],bad)
def test_empty_and_nonmonotonic_observations_rejected():
    with pytest.raises(ValueError): A.first_passive_ready_eligible([],0)
    with pytest.raises(ValueError): A.first_passive_ready_eligible([sample(2),sample(1)],0)
def test_witness_is_staticly_passive(): assert W.witness_contract()['publishers']==() and 'create_publisher' not in (D/'phase4_p4e6b_passive_readiness_witness.py').read_text()

@pytest.mark.parametrize('value,expected',[(bytes([0,1,255]),'0001ff'),(bytearray([16,32]),'1020'),([0,1,255],'0001ff'),((10,11),'0a0b')])
def test_witness_canonical_endpoint_gid_valid_shapes(value,expected):
    assert W.canonical_endpoint_gid(value)==expected

@pytest.mark.parametrize('value',[None,[],[1,-1],[256],[1.0],[[1]],object()])
def test_witness_canonical_endpoint_gid_rejects_malformed(value):
    with pytest.raises((TypeError,ValueError)): W.canonical_endpoint_gid(value)
def test_witness_is_pre_rclpy_bootstrapped():
 text=(D/'phase4_p4e6b_passive_readiness_witness.py').read_text(); assert text.index("'PROCESS_START'") < text.index('import rclpy')
def test_explicit_qos_and_no_rosout():
 text=(D/'phase4_p4e6b_passive_readiness_witness.py').read_text(); assert 'DurabilityPolicy.VOLATILE' in text and 'enable_rosout=False' in text
@pytest.mark.parametrize('old,new,event',[ (None,('n','b','n','d'),'ESTABLISHED'), (('n','b','n','d'),None,'LOST'), (('n','b','n','d'),('n','b2','n','d2'),'REBOUND')])
def test_epoch_transitions(old,new,event):
 e=W.Epoch();e.token=old; assert e.observe(new,1,2)==event; assert (e.start_ns is None)==(new is None)
def test_bool_count_requires_authority():
 e=W.Epoch();e.bool_callback();assert e.count==0;e.observe(('n','b','n','d'),1,2);e.bool_callback();assert e.count==1;e.observe(None,2,3);e.bool_callback();assert e.count==0
def test_three_root_identity_and_survival():
 ep=H.ThreeRootEpisode('t','u',1,H.Root(1,1),H.Root(2,2),H.Root(3,3));ep.worker_exited(9);assert ep.witness.alive and ep.matrix.alive and not ep.worker.alive
@pytest.mark.parametrize('states,outcome,action',[(['POST_HELPER_OBSERVATION_ACTIVE'],'TIMEOUT','RECOVER_AND_BLOCK'),(['POST_HELPER_OBSERVATION_COMPLETE','CLEANUP_VERIFIED'],'TIMEOUT','FINALIZE_CHARACTERIZATION_CLOSED'),(['CHARACTERIZATION_CLOSED'],'TIMEOUT','NONE')])
def test_crash_reconstruction(states,outcome,action): assert H.reconstruct_characterization(states,outcome)['action']==action
def test_actual_witness_schema_fixture(tmp_path):
 (tmp_path/'graph_observations.jsonl').write_text(json.dumps({'observation_monotonic_ns':4_000_000_000,'canonical_bool_publisher_count':1,'canonical_bool_node_identity':'collision_monitor_validity_monitor','canonical_bool_gid':'b'})+'\n')
 (tmp_path/'bool_observations.jsonl').write_text(json.dumps({'receipt_monotonic_ns':4_000_000_000,'value':True,'post_epoch_bool_count':2})+'\n')
 (tmp_path/'diagnostic_observations.jsonl').write_text(json.dumps({'receipt_monotonic_ns':4_000_000_000,'receipt_ros_ns':4_000_000_000,'header_ros_ns':4_000_000_000,'state':'VALID','reason_code':'VALID','healthy_stable_sec':1.,'source_age_sec':.1,'diagnostic_graph_gid':'d','epoch_start_ros_ns':1})+'\n')
 samples=A.witness_samples(tmp_path);assert A.first_passive_ready_eligible(samples,0)['event']=='PASSIVE_READY_ELIGIBLE'
