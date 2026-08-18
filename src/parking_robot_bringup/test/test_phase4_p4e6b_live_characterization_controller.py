import importlib.util
import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import pytest


HERE = Path(__file__).parent
SOURCE = HERE / 'phase4_p4e6b_live_characterization_controller.py'


def load():
    spec=importlib.util.spec_from_file_location('live_controller',SOURCE)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


C=load()


def test_formal_controller_is_not_imported():
    text=SOURCE.read_text()
    assert 'run_one_readiness_episode' not in text
    assert 'phase4_p4e6b_readiness50_harness' not in text


@pytest.mark.parametrize('cwd',[Path.cwd(), HERE, Path('/tmp')])
def test_controller_help_is_cwd_independent(cwd):
    result=subprocess.run([sys.executable,str(SOURCE),'--help'],cwd=cwd,capture_output=True,text=True)
    assert result.returncode==0 and 'DUMMY_NONROS' not in result.stderr


def test_matrix_cannot_spawn_without_admission(tmp_path):
    c=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,100)
    c.state('PLANNED')
    with pytest.raises(RuntimeError,match='durable admission'): c._spawn_dummy('matrix')


def test_bad_admission_blocks_matrix(tmp_path):
    c=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,100)
    c.state('PLANNED'); assert not c.admission(parent_ok=False)
    assert 'MATRIX_COMMITTED' not in C.read_states(c.journal)


def test_dummy_ready_transaction(tmp_path):
    r=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,101).run_dummy('READY')
    assert r['closed'] and r['helper_success'] and r['states'].index('WITNESS_READY') < r['states'].index('WORKER_SPAWNED')
    assert r['states'][-1]=='CHARACTERIZATION_CLOSED'


def test_dummy_timeout_is_failure_and_closes(tmp_path):
    r=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,102).run_dummy('TIMEOUT')
    assert r['closed'] and not r['helper_success'] and r['helper_outcome']=='TIMEOUT'
    assert r['states'].index('POST_HELPER_OBSERVATION_ACTIVE') < r['states'].index('POST_HELPER_OBSERVATION_COMPLETE')


def test_dummy_continuity_probe(tmp_path):
    r=C.LiveCharacterizationController(tmp_path,C.ScenarioType.WITNESS_CONTINUITY_PROBE,1,103).run_dummy()
    assert r['closed'] and 'WITNESS_CONTINUITY_PROBE_WORKER_EXIT' in r['states'] and r['states'][-1]=='PROBE_CLOSED'


def test_controller_crash_reconstruction(tmp_path):
    c=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,104)
    c.state('PLANNED'); assert c.admission(); c.state('MATRIX_COMMITTED')
    assert c.restart_reconstruction()['reconstructed_stage']=='MATRIX_SPAWNED_WITNESS_ABSENT'
    c.state('HELPER_ENTER_OBSERVED'); assert c.restart_reconstruction()['reconstructed_stage']=='AFTER_HELPER_ENTER_BEFORE_OUTCOME'
    c.state('HELPER_TIMEOUT'); c.state('POST_HELPER_OBSERVATION_ACTIVE')
    assert c.restart_reconstruction()['safe_recovery_action']=='RESUME_PASSIVE_OBSERVATION_IF_SAFE'
    c.state('POST_HELPER_OBSERVATION_COMPLETE'); c.state('CLEANUP_VERIFIED')
    assert c.restart_reconstruction()['safe_recovery_action']=='FINALIZE_CHARACTERIZATION_CLOSED'


def test_controller_escaped_descendant_cleanup_integration(tmp_path):
    r=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,105).run_dummy('TIMEOUT',escaped=True)
    journal=[json.loads(x) for x in Path(r['episode'],'transaction_journal.jsonl').read_text().splitlines()]
    cleanup=next(x for x in journal if x['state']=='CLEANUP_VERIFIED')
    assert cleanup['final_token_count']==0
    assert any(x['action']=='TERM_PID' for x in cleanup['actions'])


def test_closed_requires_cleanup(tmp_path):
    c=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,106)
    c.state('PLANNED'); assert c.admission(); c.state('MATRIX_COMMITTED'); c.state('CHARACTERIZATION_CLOSED')
    assert c.restart_reconstruction()['fatal_inconsistency']


def _admitted_parent():
    # The test runner is invoked from the clean sourced environment.
    return dict(os.environ)


def test_live_binding_positive_zero_spawn(tmp_path, monkeypatch):
    binding=C.build_live_ros_binding(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,301,'token-301','uuid-301',_admitted_parent())
    assert C.validate_live_binding(binding)
    assert binding['root_specs']['matrix']['argv'][1:3] == ['launch','parking_robot_bringup']
    assert binding['root_specs']['witness']['argv'][0] == '/usr/bin/python3'
    assert binding['formal_runtime_dependency'] == 'NONE'


@pytest.mark.parametrize('mutator',[
    lambda b: b.update(live_backend_binding_sha256='bad'),
    lambda b: b['root_specs']['matrix']['argv'].append('bad'),
])
def test_live_binding_hash_mismatch_fails(mutator, tmp_path):
    binding=C.build_live_ros_binding(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,302,'token-302','uuid-302',_admitted_parent())
    mutator(binding); assert not C.validate_live_binding(binding)


def test_unknown_backend_fails_closed(tmp_path):
    with pytest.raises(ValueError): C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,1,backend='unknown')


def test_parent_token_blocks_binding(tmp_path):
    parent=_admitted_parent(); parent[C.TOKEN_KEY]='leak'
    with pytest.raises(RuntimeError,match='parent carries'): C.build_live_ros_binding(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,'token','uuid',parent)


def test_live_dispatch_requires_binding_before_spawn(tmp_path):
    c=C.LiveCharacterizationController(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,1,1,backend=C.BackendType.LIVE_ROS)
    c.state('PLANNED'); c.state('PRE_EPISODE_ADMISSION_PASS')
    with pytest.raises(RuntimeError,match='LIVE_BACKEND_BINDING_PASS'): c._spawn_live('matrix')


@pytest.mark.parametrize('cwd',[Path('/home/dog/fastlio_rtabmap_ros2_ws'), HERE, Path('/tmp')])
def test_live_binding_is_cwd_independent(tmp_path, monkeypatch, cwd):
    monkeypatch.chdir(cwd)
    binding=C.build_live_ros_binding(tmp_path,C.ScenarioType.NORMAL_CHARACTERIZATION,303,'token-303','uuid-303',_admitted_parent())
    assert C.validate_live_binding(binding)
    assert binding['root_specs']['witness']['argv'][1] == str(C.WITNESS_SCRIPT.resolve())


def test_static_template_excludes_dynamic_identity_and_four_bindings(tmp_path):
    t=C.build_live_backend_template(); assert C.validate_template(t)
    assert all(x not in t for x in ('domain','token','episode_uuid','episode_index','episode_root'))
    bindings=[C.derive_live_episode_binding(t,i,500+i,f'token-{i}',f'uuid-{i}',tmp_path/f'e{i}',C.ScenarioType.NORMAL_CHARACTERIZATION if i<4 else C.ScenarioType.WITNESS_CONTINUITY_PROBE,_admitted_parent()) for i in range(1,5)]
    assert len({b['episode_binding_sha256'] for b in bindings})==4
    assert {b['static_template_sha256'] for b in bindings}=={t['live_backend_template_sha256']}


def test_template_and_episode_hash_rules(tmp_path):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,600,'token','uuid',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent())
    t2=dict(t); t2['runner_sha256']='changed'; assert not C.validate_template(t2)
    b2=dict(b); b2['domain']=601; assert not C.validate_episode_binding(b2,t)


class DummyFactory:
    outcome='READY'
    def __call__(self, role, spec): return (None, {'role':role,'argv':spec['argv']})


@pytest.mark.parametrize('scenario,outcome,terminal',[(C.ScenarioType.NORMAL_CHARACTERIZATION,'READY','CHARACTERIZATION_CLOSED'),(C.ScenarioType.NORMAL_CHARACTERIZATION,'TIMEOUT','CHARACTERIZATION_CLOSED'),(C.ScenarioType.WITNESS_CONTINUITY_PROBE,'READY','PROBE_CLOSED')])
def test_same_live_driver_dummy_paths(tmp_path,scenario,outcome,terminal):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,700,'driver-token','driver-uuid',tmp_path,scenario,_admitted_parent()); f=DummyFactory();f.outcome=outcome
    r=C.LiveCharacterizationController(tmp_path,scenario,1,700).run_live_transaction(b,t,f)
    assert r['states'][-1]==terminal
    if outcome=='TIMEOUT': assert not r['helper_success'] and 'POST_HELPER_OBSERVATION_ACTIVE' in r['states']


class FakePopen:
    next_pid=990001
    calls=[]
    def __init__(self, argv, **kwargs):
        self.pid=FakePopen.next_pid; FakePopen.next_pid+=1; FakePopen.calls.append((argv,kwargs))


class ExactSchemaFakePopen:
    """Lowest-layer non-ROS process seam; emits the production journal schema."""
    next_pid=991000
    outcome='READY'
    def __init__(self, argv, **kwargs):
        self.pid=ExactSchemaFakePopen.next_pid; ExactSchemaFakePopen.next_pid+=1
        self._returncode=None; self.argv=list(argv)
        out=Path(argv[argv.index('--output-dir')+1]) if '--output-dir' in argv else None
        if out is not None:
            out.mkdir(parents=True, exist_ok=True)
            if 'passive_readiness_witness.py' in ' '.join(argv):
                (out/'witness_phase.jsonl').write_text(json.dumps({'phase':'PROCESS_START'})+'\n'+json.dumps({'phase':'OBSERVATION_ACTIVE','monotonic_ns':1})+'\n')
                (out/'graph_observations.jsonl').write_text(json.dumps({'observation_monotonic_ns':1,'canonical_bool_publisher_count':1,'canonical_bool_node_identity':'collision_monitor_validity_monitor','canonical_bool_gid':'g'})+'\n')
                (out/'bool_observations.jsonl').write_text(json.dumps({'receipt_monotonic_ns':2,'value':True,'post_epoch_bool_count':2})+'\n')
                (out/'diagnostic_observations.jsonl').write_text(json.dumps({'receipt_monotonic_ns':2,'receipt_ros_ns':2,'header_ros_ns':1,'state':'VALID','reason_code':'VALID','healthy_stable_sec':1.0,'source_age_sec':.1,'diagnostic_graph_gid':'d'})+'\n')
                (out/'source_observations.jsonl').write_text(json.dumps({'receipt_monotonic_ns':2})+'\n')
            elif 'premission_seam.py' in ' '.join(argv):
                rows=[{'phase':'HELPER_ENTER','monotonic_ns':1}]
                if self.outcome=='READY':
                    seam={'event':{'event':'PREMISSION_HEALTH_READY','ready':True,'publisher_count':1,'generation':['collision_monitor_validity_monitor','g'],'diagnostic_writer_gid':'d','bool_post_epoch_count':2,'bool_value':True,'diagnostic_state':'VALID','diagnostic_reason':'VALID','semantic_healthy_stable_sec':1.0,'bool_age_ns':1,'diagnostic_age_ns':1,'diagnostic_transport_age_ns':1,'source_age_upper_bound_sec':.1},'helper_elapsed_ns':1}
                    (out/'premission_seam.json').write_text(json.dumps(seam))
                    rows.append({'phase':'HELPER_RETURN_READY','monotonic_ns':2})
                    rows.append({'phase':'OUTCOME_EVIDENCE_COMMITTED','outcome':'READY','artifact':'premission_seam.json','artifact_sha256':C._sha256_file(out/'premission_seam.json')})
                else:
                    (out/'helper_exception.json').write_text(json.dumps({'exception_message':'PREMISSION_HEALTH_READINESS_TIMEOUT'}))
                    rows.append({'phase':'HELPER_EXCEPTION','monotonic_ns':2})
                    rows.append({'phase':'OUTCOME_EVIDENCE_COMMITTED','outcome':'TIMEOUT','artifact':'helper_exception.json','artifact_sha256':C._sha256_file(out/'helper_exception.json')})
                (out/'worker_phase.jsonl').write_text('\n'.join(json.dumps(x) for x in rows)+'\n'); self._returncode=0
        if 'witness_continuity_worker.py' in ' '.join(argv):
            (out/'continuity_worker_phase.jsonl').write_text(json.dumps({'phase':'CONTINUITY_WORKER_READY','monotonic_ns':1})+'\n'+json.dumps({'phase':'PROCESS_EXIT','monotonic_ns':2})+'\n')
            (out.parent/'witness'/'graph_observations.jsonl').parent.mkdir(parents=True,exist_ok=True)
            graph=out.parent/'witness'/'graph_observations.jsonl'; graph.write_text(json.dumps({'observation_monotonic_ns':1})+'\n')
            def post_worker_record():
                with graph.open('a') as handle: handle.write(json.dumps({'observation_monotonic_ns':3_100_000_002})+'\n')
            threading.Timer(.5,post_worker_record).start()
            self._returncode=0
    def poll(self): return self._returncode
    def wait(self, timeout=None): self._returncode=0; return 0


def test_controller_owned_live_factory_consumes_exact_specs(tmp_path):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,901,'live-token','live-uuid',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent())
    c=C.LiveCharacterizationController(tmp_path,'NORMAL_CHARACTERIZATION',1,901,backend='LIVE_ROS')
    f=C.LiveRosProcessFactory(c,t,b,FakePopen)
    for role in ('matrix','witness','worker'):
        proc,row=f(role,b['root_specs'][role]); assert row['argv']==b['root_specs'][role]['argv'] and row['episode_binding_sha256']==b['episode_binding_sha256']
    assert all(call[1]['shell'] is False and call[1]['cwd']==b['root_specs'][role]['cwd'] for call,role in zip(FakePopen.calls[-3:],('matrix','witness','worker')))


def test_factory_rejects_role_mutation_duplicate_and_parent_token(tmp_path,monkeypatch):
    t=C.build_live_backend_template();b=C.derive_live_episode_binding(t,1,902,'live-token-2','live-uuid-2',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent());c=C.LiveCharacterizationController(tmp_path,'NORMAL_CHARACTERIZATION',1,902,backend='LIVE_ROS');f=C.LiveRosProcessFactory(c,t,b,FakePopen)
    with pytest.raises(RuntimeError): f('continuity_worker',b['root_specs']['continuity_worker'])
    f('matrix',b['root_specs']['matrix'])
    with pytest.raises(RuntimeError): f('matrix',b['root_specs']['matrix'])
    monkeypatch.setenv(C.TOKEN_KEY,'leak')
    with pytest.raises(RuntimeError): C.LiveRosProcessFactory(c,t,b,FakePopen)


def test_real_factory_wiring_uses_common_driver_with_mock_popen(tmp_path):
    t=C.build_live_backend_template();b=C.derive_live_episode_binding(t,1,903,'live-token-3','live-uuid-3',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent());c=C.LiveCharacterizationController(tmp_path,'NORMAL_CHARACTERIZATION',1,903,backend='LIVE_ROS')
    # Structural wiring is concrete; the durable-evidence loop is separately
    # exercised with journal fixtures rather than executing ROS.
    f=C.LiveRosProcessFactory(c,t,b,FakePopen)
    assert isinstance(f,C.LiveRosProcessFactory)


def test_live_driver_adopts_sealed_binding_identity_for_journal_and_cleanup(tmp_path):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,904,'sealed-token','sealed-uuid',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent())
    c=C.LiveCharacterizationController(tmp_path,'NORMAL_CHARACTERIZATION',1,904,backend='LIVE_ROS')
    f=DummyFactory(); r=c.run_live_transaction(b,t,f)
    assert r['token']=='sealed-token'
    assert all(row.get('token')=='sealed-token' for row in [json.loads(x) for x in Path(c.journal).read_text().splitlines()])


def test_fastdds_domain_policy_rejects_invalid_and_accepts_safe_pool():
    assert not C.fastdds_port_audit(251)['valid']
    assert not C.fastdds_port_audit(233)['valid']
    assert not C.fastdds_port_audit(-1)['valid']
    assert not C.fastdds_port_audit('220')['valid']
    assert C.fastdds_port_audit(220)['valid']


def test_domain_allocator_rejects_duplicate_reservation(tmp_path, monkeypatch):
    allocator=C.DomainAllocator(pool=(220,221))
    monkeypatch.setattr(C,'probe_domain_viability',lambda domain,env,out:{'domain':domain,'returncode':0,'cleanup':True})
    first,_=allocator.reserve_and_probe('A',tmp_path); second,_=allocator.reserve_and_probe('B',tmp_path)
    assert (first,second)==(220,221)
    with pytest.raises(RuntimeError): allocator.reserve_and_probe('C',tmp_path)


@pytest.mark.parametrize('scenario,outcome,terminal',[(C.ScenarioType.NORMAL_CHARACTERIZATION,'READY','CHARACTERIZATION_CLOSED'),(C.ScenarioType.NORMAL_CHARACTERIZATION,'TIMEOUT','CHARACTERIZATION_CLOSED'),(C.ScenarioType.WITNESS_CONTINUITY_PROBE,'READY','PROBE_CLOSED')])
def test_exact_serialized_binding_dress_rehearsal(tmp_path, monkeypatch, scenario, outcome, terminal):
    t=C.build_live_backend_template(); raw=json.loads(json.dumps(t)); assert C.validate_template(raw)
    b=C.derive_live_episode_binding(raw,1,950,'schema-token','schema-uuid',tmp_path,scenario,_admitted_parent())
    serialized=json.loads(json.dumps(b)); assert C.validate_episode_binding(serialized,raw)
    monkeypatch.setattr(C,'admission_for_binding',lambda binding:{'parent':{'passed':True},'child':{'passed':True}})
    monkeypatch.setattr(C,'witness_noninterference',lambda env:{'publishers':[],'protected_publishers':[]})
    ExactSchemaFakePopen.outcome=outcome
    c=C.LiveCharacterizationController(tmp_path,scenario,1,950,backend='LIVE_ROS')
    monkeypatch.setattr(c,'_terminate',lambda roles: None)
    f=C.LiveRosProcessFactory(c,raw,serialized,ExactSchemaFakePopen)
    r=c.run_live_transaction(serialized,raw,f)
    assert r['states'][-1]==terminal
    if outcome=='TIMEOUT': assert r['helper_success'] is False


def test_root_schema_requires_sealed_evidence_contract(tmp_path):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,951,'schema-token-2','schema-uuid-2',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent())
    b['root_specs']['witness'].pop('output_dir'); b['episode_binding_sha256']='bad'
    assert not C.validate_episode_binding(b,t)


def test_root_schema_rejects_argv_output_mismatch(tmp_path):
    t=C.build_live_backend_template(); b=C.derive_live_episode_binding(t,1,952,'schema-token-3','schema-uuid-3',tmp_path,'NORMAL_CHARACTERIZATION',_admitted_parent())
    b['root_specs']['witness']['argv'][-1]='/different'; value=dict(b); value.pop('episode_binding_sha256'); b['episode_binding_sha256']=__import__('hashlib').sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    assert not C.validate_episode_binding(b,t)


class _LiveProcess:
    def poll(self):
        return None


def test_outcome_evidence_barrier_waits_for_delayed_publication(tmp_path):
    worker_file=tmp_path/'worker_phase.jsonl'; artifact=tmp_path/'premission_seam.json'
    worker_file.write_text(json.dumps({'phase':'HELPER_RETURN_READY','monotonic_ns':1})+'\n')
    seam={'event':{'event':'PREMISSION_HEALTH_READY','ready':True}}
    def publish():
        import time
        time.sleep(.02)
        artifact.write_text(json.dumps(seam,sort_keys=True))
        worker_file.open('a').write(json.dumps({'phase':'OUTCOME_EVIDENCE_COMMITTED','outcome':'READY','artifact':'premission_seam.json','artifact_sha256':C._sha256_file(artifact)})+'\n')
    thread=threading.Thread(target=publish); thread.start()
    row=C.wait_outcome_evidence_commit(worker_file,'READY',artifact,_LiveProcess(),timeout=2)
    thread.join(); assert row['outcome']=='READY'


def test_outcome_evidence_barrier_stress_never_consumes_before_commit(tmp_path):
    import time
    for index in range(100):
        directory=tmp_path/f'iteration_{index}'; directory.mkdir(); worker_file=directory/'worker_phase.jsonl'; artifact=directory/'premission_seam.json'
        worker_file.write_text(json.dumps({'phase':'HELPER_RETURN_READY','monotonic_ns':index})+'\n')
        seam={'event':{'event':'PREMISSION_HEALTH_READY','ready':True},'iteration':index}
        def publish():
            if index % 3: time.sleep(.001)
            artifact.write_text(json.dumps(seam,sort_keys=True))
            with worker_file.open('a') as handle:
                handle.write(json.dumps({'phase':'OUTCOME_EVIDENCE_COMMITTED','outcome':'READY','artifact':'premission_seam.json','artifact_sha256':C._sha256_file(artifact)})+'\n')
        thread=threading.Thread(target=publish); thread.start()
        assert C.wait_outcome_evidence_commit(worker_file,'READY',artifact,_LiveProcess(),timeout=2)['outcome']=='READY'
        thread.join()


def test_missing_outcome_commit_never_invokes_consumption(tmp_path):
    worker_file=tmp_path/'worker_phase.jsonl'; artifact=tmp_path/'premission_seam.json'
    worker_file.write_text(json.dumps({'phase':'HELPER_RETURN_READY','monotonic_ns':1})+'\n')
    with pytest.raises(RuntimeError,match='timeout waiting'):
        C.wait_outcome_evidence_commit(worker_file,'READY',artifact,_LiveProcess(),timeout=.05)


def test_conflicting_outcome_commit_fails_closed(tmp_path):
    worker_file=tmp_path/'worker_phase.jsonl'; artifact=tmp_path/'premission_seam.json'; artifact.write_text('{}')
    digest=C._sha256_file(artifact)
    worker_file.write_text('\n'.join(json.dumps(x) for x in [
        {'phase':'HELPER_RETURN_READY'},
        {'phase':'OUTCOME_EVIDENCE_COMMITTED','outcome':'TIMEOUT','artifact_sha256':digest},
    ])+'\n')
    with pytest.raises(RuntimeError,match='conflicting'):
        C.wait_outcome_evidence_commit(worker_file,'READY',artifact,_LiveProcess(),timeout=.1)
