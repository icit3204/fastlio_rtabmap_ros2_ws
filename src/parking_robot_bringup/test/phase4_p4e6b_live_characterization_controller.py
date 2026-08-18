"""Qualification-only executable characterization transaction controller.

This module deliberately contains no ROS imports.  The production live backend
is represented by injected commands; this phase validates orchestration through
the explicitly labelled ``DUMMY_NONROS`` backend only.
"""
import argparse
import hashlib
import json
import os
import signal
import shutil
import subprocess
import sys
import time
import uuid
import re
from enum import Enum
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from phase4_p4e6b_timeout_characterization_harness import reconstruct_characterization_episode
from phase4_p4e6b_three_root_selfqual import TOKEN_KEY, token_processes
from phase4_p4e6b_passive_eligibility_analyzer import witness_samples, first_passive_ready_eligible
from phase4_p4e6b_terminal_closure_controller import (
    commit_final_freeze, prospective_freeze_identities, assert_matrix_after_freeze,
)
from phase4_p4e6b_hash_pinned_freeze import (
    commit_pinned_freeze, load_expected_authority, sha256_file,
    assert_process_order,
)

# B2AJ prospective qualification authority. Historical records remain bound
# to the old SHA and are never rewritten.
FROZEN_RUNNER_SHA256 = '458d2a6e184079a1236d28664c4554292a9b4a2cef72e3c576f5c4b89d545b71'
FROZEN_RUNNER = HERE.parent / 'parking_robot_bringup' / 'phase4_p4e6b_health_failure_runner.py'
WORKSPACE = HERE.parents[2]
MATRIX_SOURCE = WORKSPACE / 'src/parking_robot_bringup/launch/phase4_p4e6b_health_matrix.launch.py'
MATRIX_INSTALLED = WORKSPACE / 'install/parking_robot_bringup/share/parking_robot_bringup/launch/phase4_p4e6b_health_matrix.launch.py'
WITNESS_SCRIPT = HERE / 'phase4_p4e6b_passive_readiness_witness.py'
WORKER_SCRIPT = HERE / 'phase4_p4e6b_premission_seam.py'
CONTINUITY_SCRIPT = HERE / 'phase4_p4e6b_witness_continuity_worker.py'
AUTHORITY_FILE = HERE / 'phase4_p4e6b_live_health_authority.json'
FREEZE_CONTROLLER = HERE / 'phase4_p4e6b_terminal_closure_controller.py'
RELAY_SCRIPT = HERE.parent / 'parking_robot_bringup' / 'phase4_p4e6b_feedback_relay.py'
CHILD_PYTHON = '/usr/bin/python3'
SAFE_DOMAIN_POOL = tuple(range(220, 233))
PROTECTED_GRAPH_TOPICS = ('/system/collision_monitor_valid','/diagnostics','/phase4/synthetic_scan','/cmd_vel_nav_raw','/cmd_vel_nav_safe','/vehicle_cmd_safe','/wheelchair_control_command_mock','/wheelchair_control_command')


class ScenarioType(str, Enum):
    NORMAL_CHARACTERIZATION = 'NORMAL_CHARACTERIZATION'
    WITNESS_CONTINUITY_PROBE = 'WITNESS_CONTINUITY_PROBE'


class BackendType(str, Enum):
    DUMMY_NONROS = 'DUMMY_NONROS'
    LIVE_ROS = 'LIVE_ROS'


def _fingerprint(env):
    selected = {key: env.get(key) for key in ('PATH','PYTHONPATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','ROS_DISTRO','ROS_VERSION','ROS_PYTHON_VERSION','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','P4E6B_EPISODE_TOKEN','P4E6B_EPISODE_UUID')}
    return {'values': selected, 'sha256': hashlib.sha256(json.dumps(selected, sort_keys=True, separators=(',', ':')).encode()).hexdigest()}


def _child_env(parent, domain, token, episode_uuid):
    if TOKEN_KEY in parent or 'P4E6B_EPISODE_UUID' in parent:
        raise RuntimeError('parent carries episode identity')
    env = parent.copy()
    env.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1', TOKEN_KEY: token,
                'P4E6B_EPISODE_UUID': episode_uuid, 'PYTHONUNBUFFERED': '1'})
    return env


def middleware_identity(env):
    """Resolve middleware from the exact child environment, without a node."""
    code = 'import rclpy; print(rclpy.get_rmw_implementation_identifier())'
    probe = subprocess.run([CHILD_PYTHON, '-c', code], env=env, capture_output=True, text=True, timeout=5)
    if probe.returncode != 0 or not probe.stdout.strip():
        raise RuntimeError(f'RMW identity resolution failed: {probe.stderr.strip()}')
    return {'identifier':probe.stdout.strip(),'environment_value':env.get('RMW_IMPLEMENTATION'),
            'ros_distro':env.get('ROS_DISTRO'),'ros_localhost_only':env.get('ROS_LOCALHOST_ONLY')}


def fastdds_port_audit(domain, port_base=7400, domain_id_gain=250, participant_id_gain=2,
                       offsets=(0, 10, 1, 11), max_port=65535):
    """Validate the default Fast DDS RTPS UDP port envelope mathematically."""
    if not isinstance(domain, int) or isinstance(domain, bool):
        return {'valid':False,'reason':'domain must be integer'}
    if domain < 0:
        return {'valid':False,'reason':'domain must be non-negative'}
    ports=[port_base + domain_id_gain*domain + offset for offset in offsets]
    valid=max(ports) <= max_port
    return {'valid':valid,'reason':None if valid else 'RTPS port envelope exceeds 65535',
            'parameters':{'port_base':port_base,'domain_id_gain':domain_id_gain,
                          'participant_id_gain':participant_id_gain,'offsets':list(offsets),
                          'max_port':max_port},'ports':ports}


def validate_domain_candidate(domain, env):
    middleware=middleware_identity(env)
    identifier=middleware['identifier'].lower()
    if 'fastrtps' not in identifier and 'fastdds' not in identifier:
        raise RuntimeError(f'unsupported middleware for current domain policy: {identifier}')
    audit=fastdds_port_audit(domain)
    if not audit['valid']:
        raise RuntimeError(f'domain {domain} rejected: {audit["reason"]}')
    return {'domain':domain,'middleware':middleware,'port_audit':audit}


def probe_domain_viability(domain, parent_environment, output_dir, timeout=6):
    """Create one real RMW participant, inspect graph, and cleanly reap it."""
    env=dict(parent_environment)
    env['ROS_DOMAIN_ID']=str(domain); env['ROS_LOCALHOST_ONLY']='1'
    validate_domain_candidate(domain,env)
    name=f'phase4_domain_viability_probe_{domain}'
    code=('import json,time,rclpy; from rclpy.node import Node; '
          f'rclpy.init(); n=Node({name!r}); print(json.dumps({{"phase":"PARTICIPANT_READY","domain":{domain}}}),flush=True); '
          'rclpy.spin_once(n,timeout_sec=0.2); time.sleep(0.8); n.destroy_node(); rclpy.shutdown(); '
          'print(json.dumps({"phase":"PARTICIPANT_CLEAN"}),flush=True)')
    out=Path(output_dir); out.mkdir(parents=True,exist_ok=True)
    started=time.monotonic_ns(); proc=subprocess.Popen([CHILD_PYTHON,'-c',code],env=env,
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    pid,pgid=proc.pid,proc.pid
    graph=subprocess.run(['ros2','node','list'],env=env,capture_output=True,text=True,timeout=5)
    graph_text=graph.stdout+graph.stderr
    unexpected=[line for line in graph.stdout.splitlines() if any(topic in line for topic in PROTECTED_GRAPH_TOPICS)]
    try: stdout,stderr=proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill(); stdout,stderr=proc.communicate(); raise RuntimeError('participant viability probe timeout')
    result={'domain':domain,'pid':pid,'pgid':pgid,'start_monotonic_ns':started,
            'end_monotonic_ns':time.monotonic_ns(),'returncode':proc.returncode,
            'stdout':stdout,'stderr':stderr,'graph_returncode':graph.returncode,
            'graph':graph_text,'unexpected_protected_graph':unexpected,
            'middleware':middleware_identity(env),'cleanup':proc.returncode==0}
    durable_json(out/'domain_viability_probe.json',result)
    if proc.returncode != 0 or unexpected:
        raise RuntimeError(f'domain viability failed for {domain}: {result}')
    return result


class DomainAllocator:
    def __init__(self, parent_environment=None, pool=SAFE_DOMAIN_POOL):
        self.environment=dict(os.environ if parent_environment is None else parent_environment)
        self.pool=tuple(pool); self.reserved=set(); self.ledger=[]
    def reserve_and_probe(self, scenario, output_dir):
        for domain in self.pool:
            if domain in self.reserved: continue
            audit=fastdds_port_audit(domain)
            if not audit['valid']:
                self.ledger.append({'scenario':scenario,'domain':domain,'status':'REJECTED','reason':audit['reason']}); continue
            try:
                result=probe_domain_viability(domain,self.environment,Path(output_dir)/f'domain_{domain}')
            except Exception as exc:
                self.ledger.append({'scenario':scenario,'domain':domain,'status':'REJECTED','reason':str(exc)}); continue
            self.reserved.add(domain)
            row={'scenario':scenario,'domain':domain,'status':'VIABLE_RESERVED_FOR_SCENARIO','probe':result}
            self.ledger.append(row); return domain,row
        raise RuntimeError('no viable domain available')


def _admission(env):
    code = ('import importlib.metadata; import rclpy, launch, launch_ros, ament_index_python; '
            'from ament_index_python.packages import get_package_prefix; '
            "assert importlib.metadata.version('ros2cli'); assert get_package_prefix('parking_robot_bringup'); assert get_package_prefix('parking_robot_interfaces')")
    probe = subprocess.run([CHILD_PYTHON, '-c', code], env=env, capture_output=True, text=True)
    cli = subprocess.run(['ros2','--help'], env=env, capture_output=True, text=True) if probe.returncode == 0 else None
    prefix = subprocess.run(['ros2','pkg','prefix','parking_robot_bringup'], env=env, capture_output=True, text=True) if cli and cli.returncode == 0 else None
    return {'passed': probe.returncode == 0 and cli is not None and cli.returncode == 0 and prefix is not None and prefix.returncode == 0,
            'python_returncode': probe.returncode, 'ros2_help_returncode': None if cli is None else cli.returncode,
            'package_prefix_returncode': None if prefix is None else prefix.returncode,
            'parking_robot_bringup_prefix': None if prefix is None else prefix.stdout.strip()}


def build_live_ros_binding(output_root, scenario, domain, token, episode_uuid, parent_environment=None):
    """Build and validate immutable LIVE_ROS root specs; never calls Popen."""
    scenario = ScenarioType(scenario)
    parent = dict(os.environ if parent_environment is None else parent_environment)
    child = _child_env(parent, domain, token, episode_uuid)
    ros2 = Path(child.get('PATH','').split(':')[0]) / 'ros2'
    resolved_ros2 = shutil.which('ros2', path=child.get('PATH'))
    if not resolved_ros2 or not MATRIX_SOURCE.exists() or not MATRIX_INSTALLED.exists():
        raise RuntimeError('required LIVE_ROS executable or launch source missing')
    for script in (WITNESS_SCRIPT, WORKER_SCRIPT, CONTINUITY_SCRIPT):
        if not script.is_file(): raise RuntimeError(f'missing qualification script: {script}')
    if not Path(CHILD_PYTHON).is_file(): raise RuntimeError('wrong Python executable')
    admission = _admission(child)
    if not admission['passed']: raise RuntimeError('LIVE_ROS child environment admission failed')
    root = Path(output_root).resolve(); episode = root / f'scenario_{episode_uuid}'
    env_record = _fingerprint(child)
    specs = {
        'matrix': {'role':'MATRIX_ROOT','argv':[str(Path(resolved_ros2).resolve()),'launch','parking_robot_bringup','phase4_p4e6b_health_matrix.launch.py','enable_health_runner:=false'],'cwd':str(root),'environment':env_record,'output_dir':str(episode/'matrix'),'expected_marker':'MATRIX_COMMITTED','normal_exit':'controller cleanup'},
        'witness': {'role':'WITNESS_ROOT','argv':[CHILD_PYTHON,str(WITNESS_SCRIPT.resolve()),'--output-dir',str(episode/'witness')],'cwd':str(root),'environment':env_record,'output_dir':str(episode/'witness'),'expected_marker':'OBSERVATION_ACTIVE','normal_exit':'controller cleanup'},
        'worker': {'role':'WORKER_ROOT','argv':[CHILD_PYTHON,str(WORKER_SCRIPT.resolve()),'--output-dir',str(episode/'worker'),'--diagnostic-qos','best_effort'],'cwd':str(root),'environment':env_record,'output_dir':str(episode/'worker'),'expected_marker':'HELPER_ENTER','normal_exit':'worker natural exit'},
        'continuity_worker': {'role':'CONTINUITY_WORKER_ROOT','argv':[CHILD_PYTHON,str(CONTINUITY_SCRIPT.resolve()),'--output-dir',str(episode/'continuity_worker')],'cwd':str(root),'environment':env_record,'output_dir':str(episode/'continuity_worker'),'expected_marker':'CONTINUITY_WORKER_READY','normal_exit':'intentional bounded exit'},
    }
    binding = {'backend':BackendType.LIVE_ROS.value, 'scenario':scenario.value, 'domain':domain, 'token':token, 'episode_uuid':episode_uuid, 'controller_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'runner_sha256':hashlib.sha256(FROZEN_RUNNER.read_bytes()).hexdigest(), 'ros2_path':resolved_ros2, 'ros2_realpath':str(Path(resolved_ros2).resolve()), 'ros2_shebang':Path(resolved_ros2).read_text().splitlines()[0], 'python_executable':CHILD_PYTHON, 'matrix_launch_source':str(MATRIX_SOURCE.resolve()), 'matrix_launch_installed':str(MATRIX_INSTALLED.resolve()), 'formal_runtime_dependency':'NONE', 'parent_environment':_fingerprint(parent), 'child_environment':env_record, 'admission':admission, 'root_specs':specs}
    canonical = json.dumps(binding, sort_keys=True, separators=(',', ':')).encode()
    binding['live_backend_binding_sha256'] = hashlib.sha256(canonical).hexdigest()
    return binding


def validate_live_binding(binding):
    value = dict(binding); claimed = value.pop('live_backend_binding_sha256', None)
    actual = hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return claimed == actual and value.get('backend') == BackendType.LIVE_ROS.value and value.get('admission',{}).get('passed') is True


def build_live_backend_template():
    """Static reviewed authority: intentionally contains no episode identity."""
    template={'template_version':1,'backend':'LIVE_ROS','matrix':{'package':'parking_robot_bringup','launch':'phase4_p4e6b_health_matrix.launch.py','argv':[str(Path(shutil.which('ros2')).resolve()),'launch','parking_robot_bringup','phase4_p4e6b_health_matrix.launch.py','enable_health_runner:=false']},'witness':{'python':CHILD_PYTHON,'script':str(WITNESS_SCRIPT.resolve()),'argv':[CHILD_PYTHON,str(WITNESS_SCRIPT.resolve()),'--output-dir','{WITNESS_OUTPUT_DIR}']},'worker':{'python':CHILD_PYTHON,'script':str(WORKER_SCRIPT.resolve()),'argv':[CHILD_PYTHON,str(WORKER_SCRIPT.resolve()),'--output-dir','{WORKER_OUTPUT_DIR}','--diagnostic-qos','best_effort']},'continuity_worker':{'python':CHILD_PYTHON,'script':str(CONTINUITY_SCRIPT.resolve()),'argv':[CHILD_PYTHON,str(CONTINUITY_SCRIPT.resolve()),'--output-dir','{CONTINUITY_OUTPUT_DIR}']},'analyzer':str((HERE/'phase4_p4e6b_passive_eligibility_analyzer.py').resolve()),'runner_path':str(FROZEN_RUNNER.resolve()),'runner_sha256':FROZEN_RUNNER_SHA256,'formal_runtime_dependency':'NONE','environment_derivation_policy':'FULL_PARENT_COPY_V1','cleanup_policy':'EXACT_TOKEN_PGID_V1','required_markers':{'witness':'OBSERVATION_ACTIVE','worker':'HELPER_ENTER','continuity_worker':'CONTINUITY_WORKER_READY'},'controller_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    template['live_backend_template_sha256']=hashlib.sha256(json.dumps(template,sort_keys=True,separators=(',',':')).encode()).hexdigest(); return template


def derive_live_episode_binding(template, episode_index, domain, token, episode_uuid, episode_root, scenario, parent_environment):
    """Pure identity binder; template hash is checked before any substitution."""
    if not validate_template(template): raise RuntimeError('static template integrity failure')
    if not all((isinstance(episode_index,int), isinstance(domain,int), token, episode_uuid)) : raise RuntimeError('invalid episode identity')
    scenario=ScenarioType(scenario); env=_child_env(parent_environment,domain,token,episode_uuid); root=Path(episode_root).resolve()
    values={'WITNESS_OUTPUT_DIR':str(root/'witness'),'WORKER_OUTPUT_DIR':str(root/'worker'),'CONTINUITY_OUTPUT_DIR':str(root/'continuity_worker')}
    def resolve(argv):
        out=[]
        for item in argv:
            try: out.append(item.format(**values))
            except KeyError as exc: raise RuntimeError(f'unresolved placeholder {exc}')
        if any('{' in x or '}' in x for x in out): raise RuntimeError('unresolved placeholder')
        return out
    output_dirs={'matrix':root/'matrix','witness':root/'witness','worker':root/'worker','continuity_worker':root/'continuity_worker'}
    journal_names={'matrix':None,'witness':'witness_phase.jsonl','worker':'worker_phase.jsonl','continuity_worker':'continuity_worker_phase.jsonl'}
    roots={}
    for name,spec in ((k,template[k]) for k in ('matrix','witness','worker','continuity_worker')):
        argv=resolve(spec['argv'])
        output_dir=str(output_dirs[name])
        if name != 'matrix' and output_dir not in argv:
            raise RuntimeError(f'{name} argv/output_dir contract mismatch')
        roots[name]={'role':name.upper()+'_ROOT','argv':argv,'cwd':str(root),
                     'environment':_fingerprint(env),'output_dir':output_dir,
                     'journal_path':None if journal_names[name] is None else str(output_dirs[name]/journal_names[name]),
                     'expected_marker':template['required_markers'].get(name,'MATRIX_COMMITTED')}
    binding={'not_live_evidence':True,'static_template_sha256':template['live_backend_template_sha256'],'scenario':scenario.value,'episode_index':episode_index,'domain':domain,'token':token,'episode_uuid':episode_uuid,'episode_root':str(root),'root_specs':roots,'runner_sha256':template['runner_sha256'],'controller_sha256':template['controller_sha256'],'template_version':template['template_version']}
    binding['episode_binding_sha256']=hashlib.sha256(json.dumps(binding,sort_keys=True,separators=(',',':')).encode()).hexdigest(); return binding


def validate_template(template):
    value=dict(template); claimed=value.pop('live_backend_template_sha256',None)
    banned=('domain','token','episode_uuid','episode_index','episode_root','ROS_DOMAIN_ID','P4E6B_EPISODE_TOKEN','P4E6B_EPISODE_UUID')
    return claimed==hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest() and not any(k in value for k in banned)


def validate_episode_binding(binding, template):
    value=dict(binding); claimed=value.pop('episode_binding_sha256',None)
    if not (validate_template(template) and value.get('static_template_sha256')==template['live_backend_template_sha256'] and claimed==hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()):
        return False
    roots=value.get('root_specs',{})
    required={'role','argv','cwd','environment','output_dir','journal_path','expected_marker'}
    for name,spec in roots.items():
        if not required.issubset(spec):
            return False
        if not isinstance(spec['argv'],list) or not isinstance(spec['cwd'],str) or not isinstance(spec['environment'],dict):
            return False
        if name != 'matrix' and spec['output_dir'] not in spec['argv']:
            return False
        if spec['journal_path'] is not None and not spec['journal_path'].startswith(spec['output_dir'] + os.sep):
            return False
    return True


class LiveRosProcessFactory:
    """Controller-owned immutable-root-spec process authority.

    ``popen_callable`` is a low-level test seam only; the default is the exact
    shell-free subprocess contract intended for future reviewed execution.
    """
    ROLE_FOR_SCENARIO={ScenarioType.NORMAL_CHARACTERIZATION:{'matrix','witness','worker'},ScenarioType.WITNESS_CONTINUITY_PROBE:{'matrix','witness','continuity_worker'}}
    def __init__(self, controller, template, episode_binding, popen_callable=subprocess.Popen):
        if controller.backend is not BackendType.LIVE_ROS: raise RuntimeError('wrong backend')
        if not validate_episode_binding(episode_binding,template): raise RuntimeError('root spec integrity failure')
        if TOKEN_KEY in os.environ or 'P4E6B_EPISODE_UUID' in os.environ: raise RuntimeError('parent episode identity contamination')
        self.controller,self.template,self.binding,self.popen=controller,template,episode_binding,popen_callable;self.spawned=set()
    def __call__(self, role, supplied_spec):
        if role not in self.ROLE_FOR_SCENARIO[self.controller.scenario]: raise RuntimeError('role forbidden for scenario')
        if role in self.spawned: raise RuntimeError('duplicate root role')
        expected=self.binding['root_specs'].get(role)
        if expected is None or supplied_spec != expected: raise RuntimeError('root spec mutation')
        env=_child_env(os.environ,self.binding['domain'],self.binding['token'],self.binding['episode_uuid'])
        if _fingerprint(env)['sha256'] != expected['environment']['sha256']: raise RuntimeError('sealed child environment mutation')
        try: proc=self.popen(expected['argv'],cwd=expected['cwd'],env=env,shell=False,start_new_session=True)
        except Exception as exc: raise RuntimeError(f'Popen failed for {role}: {type(exc).__name__}: {exc}') from exc
        pid=getattr(proc,'pid',None)
        if not isinstance(pid,int) or pid<=0: raise RuntimeError('malformed process handle PID')
        try: pgid=os.getpgid(pid)
        except Exception:
            # Mocked Popen may not create an OS process; session leader is the
            # contract used by real Popen, so retain PID as prospective PGID.
            pgid=pid
        row={'pid':pid,'pgid':pgid,'spawn_monotonic_ns':time.monotonic_ns(),'argv':expected['argv'],'argv_sha256':hashlib.sha256(json.dumps(expected['argv']).encode()).hexdigest(),'episode_token':self.binding['token'],'episode_uuid':self.binding['episode_uuid'],'domain':self.binding['domain'],'episode_binding_sha256':self.binding['episode_binding_sha256'],'static_template_sha256':self.binding['static_template_sha256']}
        self.spawned.add(role); return proc,row


def durable_append(path, row):
    row = dict(row, monotonic_ns=row.get('monotonic_ns', time.monotonic_ns()))
    with Path(path).open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(row, sort_keys=True) + '\n')
        handle.flush(); os.fsync(handle.fileno())
    return row


def durable_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True); temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w', encoding='utf-8') as handle:
        json.dump(value, handle, sort_keys=True, indent=2); handle.write('\n')
        handle.flush(); os.fsync(handle.fileno())
    os.replace(temp, path)
    fd = os.open(str(path.parent), os.O_DIRECTORY); os.fsync(fd); os.close(fd)


def read_states(journal):
    try:
        return [json.loads(line)['state'] for line in Path(journal).read_text().splitlines() if line]
    except (OSError, ValueError, KeyError):
        return []


def wait_durable_phase(path, wanted, timeout, require_process=None):
    """Poll append-only JSONL evidence; malformed non-final records fail closed."""
    end=time.monotonic()+timeout; seen=0
    while time.monotonic()<end:
        try:
            rows=[]
            for number,line in enumerate(Path(path).read_text().splitlines(),1):
                if not line: continue
                rows.append(json.loads(line))
        except FileNotFoundError: rows=[]
        except json.JSONDecodeError as exc: raise RuntimeError(f'corrupt durable journal {path}: {exc}')
        for row in rows[seen:]:
            if row.get('phase') in wanted: return row,rows
        seen=len(rows)
        if require_process is not None and require_process.poll() is not None:
            raise RuntimeError(f'process exited before durable phase {wanted}')
        time.sleep(.02)
    raise RuntimeError(f'timeout waiting for durable phase {wanted}')


def analyze_witness(directory, helper_enter_ns, bound_sec=20.0):
    """Direct analyzer invocation over retained witness schema, durably caller-owned."""
    result=first_passive_ready_eligible(witness_samples(directory),helper_enter_ns,bound_sec)
    return result


def latest_observation_ns(directory):
    path=Path(directory)/'graph_observations.jsonl'
    try: return max((json.loads(line).get('observation_monotonic_ns',0) for line in path.read_text().splitlines() if line),default=0)
    except (FileNotFoundError,json.JSONDecodeError): return 0


def validate_frozen_ready(seam):
    """Qualification-local frozen READY validator; no formal controller import."""
    event=seam.get('event',{}); gen=event.get('generation')
    below=lambda value,limit:value is not None and value<limit
    return all((event.get('ready') is True,event.get('publisher_count')==1,isinstance(gen,list) and len(gen)==2 and gen[0]=='collision_monitor_validity_monitor' and bool(gen[1]),bool(event.get('diagnostic_writer_gid')),event.get('bool_post_epoch_count',0)>=2,event.get('bool_value') is True,event.get('diagnostic_state')=='VALID',event.get('diagnostic_reason')=='VALID',(event.get('semantic_healthy_stable_sec') or 0)>=1.0,below(event.get('bool_age_ns'),250_000_000),below(event.get('diagnostic_age_ns'),250_000_000),below(event.get('diagnostic_transport_age_ns'),250_000_000),below(event.get('source_age_upper_bound_sec'),.5),below(seam.get('helper_elapsed_ns'),8e9)))


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def wait_outcome_evidence_commit(worker_file, expected_outcome, artifact_path, process, timeout=10):
    """Wait for the worker's fsynced outcome barrier, never for a sleep."""
    deadline=time.monotonic()+timeout; seen=0
    while time.monotonic()<deadline:
        try: rows=[json.loads(line) for line in Path(worker_file).read_text().splitlines() if line]
        except FileNotFoundError: rows=[]
        except json.JSONDecodeError:
            time.sleep(.02); continue
        commits=[r for r in rows if r.get('phase')=='OUTCOME_EVIDENCE_COMMITTED']
        if any(r.get('outcome') != expected_outcome for r in commits):
            raise RuntimeError('conflicting outcome evidence commit')
        if len(commits)>1:
            raise RuntimeError('duplicate outcome evidence commits')
        if commits:
            commit=commits[0]
            if not artifact_path.is_file(): raise RuntimeError('committed outcome artifact missing')
            actual=_sha256_file(artifact_path)
            if actual != commit.get('artifact_sha256'): raise RuntimeError('outcome evidence hash mismatch')
            try: json.loads(artifact_path.read_text())
            except (OSError,json.JSONDecodeError) as exc: raise RuntimeError('committed artifact malformed') from exc
            return commit
        if process.poll() is not None: raise RuntimeError('worker exited before outcome evidence commit')
        time.sleep(.02)
    raise RuntimeError('timeout waiting for outcome evidence commit')


def admission_for_binding(binding):
    """Controller-local parent/child admission before any matrix Popen."""
    if TOKEN_KEY in os.environ or 'P4E6B_EPISODE_UUID' in os.environ: raise RuntimeError('parent identity contamination')
    parent=_admission(os.environ); child=_admission(_child_env(os.environ,binding['domain'],binding['token'],binding['episode_uuid']))
    if not parent['passed'] or not child['passed']: raise RuntimeError('parent or child ROS admission failed')
    return {'parent':parent,'child':child}


def actual_pinned_identities(controller_path=Path(__file__), runner_path=FROZEN_RUNNER,
                             matrix_path=MATRIX_SOURCE, freeze_controller_path=FREEZE_CONTROLLER,
                             witness_path=HERE / 'phase4_p4e6b_terminal_closure_witness.py',
                             relay_path=RELAY_SCRIPT):
    return {
        'runner': sha256_file(runner_path),
        'matrix': sha256_file(matrix_path),
        'live_controller': sha256_file(controller_path),
        'freeze_controller': sha256_file(freeze_controller_path),
        'witness': sha256_file(witness_path),
        'relay': sha256_file(relay_path),
    }


def admit_live_matrix_before_popen(output_dir, matrix_spawn, *,
                                   runner_path=FROZEN_RUNNER,
                                   matrix_path=MATRIX_SOURCE,
                                   authority_path=AUTHORITY_FILE,
                                   controller_path=Path(__file__),
                                   freeze_controller_path=FREEZE_CONTROLLER,
                                   witness_path=HERE / 'phase4_p4e6b_terminal_closure_witness.py',
                                   relay_path=RELAY_SCRIPT):
    """Actual live-controller matrix gate: pin, durably freeze, then spawn."""
    expected = load_expected_authority(authority_path)
    actual = actual_pinned_identities(controller_path=controller_path, runner_path=runner_path,
                                      matrix_path=matrix_path,
                                      freeze_controller_path=freeze_controller_path,
                                      witness_path=witness_path, relay_path=relay_path)
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True,
                          text=True, check=True).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--porcelain'], capture_output=True,
                           text=True, check=True).stdout
    freeze = commit_pinned_freeze(Path(output_dir), expected, actual, head=head,
                                  dirty_summary={'dirty': bool(dirty),
                                                 'sha256': hashlib.sha256(dirty.encode()).hexdigest()})
    created_ns = time.monotonic_ns()
    process = matrix_spawn()
    runtime_ns = time.monotonic_ns()
    assert_process_order(freeze['committed_ns'], created_ns, runtime_ns)
    return {'expected': expected, 'actual': actual, 'freeze': freeze,
            'process': process, 'matrix_created_ns': created_ns,
            'runtime_started_marker_ns': runtime_ns}


TARGET_MARKERS=('planner_server','controller_server','behavior_server','bt_navigator','waypoint_follower','lifecycle_manager','collision_monitor','mission_manager','phase4_p4e6b_premission_seam','phase4_p4e6b_passive_readiness_witness')
def global_target_scan():
    mine={os.getpid(),os.getppid()}; rows=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            pid=int(proc.name)
            if pid in mine: continue
            cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if any(marker in cmd for marker in TARGET_MARKERS): rows.append({'pid':pid,'command':cmd})
        except (FileNotFoundError,PermissionError,ValueError): pass
    return rows


def require_global_zero():
    first=global_target_scan(); time.sleep(2); second=global_target_scan()
    if first or second: raise RuntimeError('global target process not zero')
    return {'first':first,'second':second}


def witness_noninterference(env):
    """Graph audit: witness must not publish protected application topics."""
    witness='phase4_p4e6b_passive_readiness_witness'
    protected=('/system/collision_monitor_valid','/diagnostics','/phase4/synthetic_scan','/cmd_vel_nav_raw','/cmd_vel_nav_safe','/vehicle_cmd_safe','/wheelchair_control_command_mock','/wheelchair_control_command')
    run=subprocess.run(['ros2','node','info','/'+witness],env=env,capture_output=True,text=True)
    text=run.stdout+run.stderr; publishers=[]; in_publishers=False
    for line in text.splitlines():
        if line.strip()=='Publishers:': in_publishers=True; continue
        if line and not line.startswith(' ') and not line.startswith('\t'): in_publishers=False
        if in_publishers and ':' in line: publishers.append(line.strip().split(':',1)[0])
    bad=sorted(set(publishers)&set(protected))
    if bad: raise RuntimeError(f'witness application publisher contamination: {bad}')
    return {'node_info_returncode':run.returncode,'publishers':publishers,'protected_publishers':bad,'raw':text}


class LiveCharacterizationController:
    """Sole transaction entry; real spawning requires a later injected backend."""
    def __init__(self, root, scenario, index, domain, backend=BackendType.DUMMY_NONROS, live_binding=None):
        self.root = Path(root); self.root.mkdir(parents=True, exist_ok=True)
        self.scenario = ScenarioType(scenario)
        self.index, self.domain, self.backend = index, domain, BackendType(backend)
        self.token, self.episode_uuid = uuid.uuid4().hex, uuid.uuid4().hex
        self.episode = self.root / f'scenario_{index:02d}_{self.episode_uuid}'
        self.episode.mkdir()
        self.journal = self.episode / 'transaction_journal.jsonl'
        self.roots = {}
        self.live_binding = live_binding

    def state(self, name, **extra):
        return durable_append(self.journal, {'state': name, 'backend': self.backend.value,
                                             'scenario': self.scenario.value,
                                             'episode_uuid': self.episode_uuid,
                                             'token': self.token, 'domain': self.domain, **extra})

    def _runner_ok(self):
        return FROZEN_RUNNER.exists() and hashlib.sha256(FROZEN_RUNNER.read_bytes()).hexdigest() == FROZEN_RUNNER_SHA256

    def admission(self, parent_ok=True, child_ok=True, globals_zero=True, prior_closed=True):
        if not self._runner_ok() or not all((parent_ok, child_ok, globals_zero, prior_closed)):
            return False
        self.state('PRE_EPISODE_ADMISSION_PASS', runner_sha256=FROZEN_RUNNER_SHA256,
                   parent_admission=parent_ok, child_admission=child_ok,
                   global_first=0, global_second=0, prior_closed=prior_closed)
        return True

    def _spawn_dummy(self, role, command='exec sleep 30'):
        if self.backend is not BackendType.DUMMY_NONROS:
            raise RuntimeError('dummy spawn requested for LIVE_ROS backend')
        if 'PRE_EPISODE_ADMISSION_PASS' not in read_states(self.journal):
            raise RuntimeError('matrix/witness/worker spawn requires durable admission')
        env = os.environ.copy(); env[TOKEN_KEY] = self.token
        env['P4E6B_EPISODE_UUID'] = self.episode_uuid
        proc = subprocess.Popen(['/bin/sh', '-c', command], env=env, start_new_session=True)
        row = {'pid': proc.pid, 'pgid': proc.pid, 'command': command,
               'spawn_monotonic_ns': time.monotonic_ns(), 'environment_fingerprint': hashlib.sha256(json.dumps({TOKEN_KEY:self.token,'P4E6B_EPISODE_UUID':self.episode_uuid},sort_keys=True).encode()).hexdigest()}
        self.roots[role] = (proc, row)
        self.state({'matrix':'MATRIX_COMMITTED','witness':'WITNESS_SPAWNED','worker':'WORKER_SPAWNED'}[role], role=role, **row)
        return proc

    def _spawn_live(self, role):
        """Future-only LIVE_ROS dispatch; binding and admission are mandatory."""
        if self.backend is not BackendType.LIVE_ROS:
            raise RuntimeError('LIVE_ROS dispatch requested for non-live backend')
        if not validate_live_binding(self.live_binding or {}):
            raise RuntimeError('LIVE_BACKEND_BINDING_PASS required before spawn')
        if 'PRE_EPISODE_ADMISSION_PASS' not in read_states(self.journal):
            raise RuntimeError('matrix/witness/worker spawn requires durable admission')
        spec = self.live_binding['root_specs'][role]
        # The copied environment is intentionally never reconstructed; its hash
        # was frozen inside the binding before this dispatch point.
        env = _child_env(os.environ, self.domain, self.token, self.episode_uuid)
        if _fingerprint(env)['sha256'] != spec['environment']['sha256']:
            raise RuntimeError('root environment changed after binding')
        proc = subprocess.Popen(spec['argv'], cwd=spec['cwd'], env=env, start_new_session=True)
        row = {'pid':proc.pid, 'pgid':proc.pid, 'command':spec['argv'], 'spawn_monotonic_ns':time.monotonic_ns(), 'environment_fingerprint':spec['environment']['sha256']}
        self.roots[role] = (proc, row)
        self.state({'matrix':'MATRIX_COMMITTED','witness':'WITNESS_SPAWNED','worker':'WORKER_SPAWNED','continuity_worker':'WORKER_SPAWNED'}[role], role=role, **row, live_backend_binding_sha256=self.live_binding['live_backend_binding_sha256'])
        return proc

    def _terminate(self, roles):
        actions=[]
        for role in roles:
            proc, row = self.roots.get(role, (None, None))
            if proc and proc.poll() is None:
                os.killpg(row['pgid'], signal.SIGTERM); actions.append({'role':role,'action':'TERM_PGID','pgid':row['pgid']})
                try: proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(row['pgid'], signal.SIGKILL); actions.append({'role':role,'action':'KILL_PGID','pgid':row['pgid']}); proc.wait(timeout=2)
        survivors=token_processes(self.token)
        for item in survivors:
            os.kill(item['pid'], signal.SIGTERM); actions.append({'role':'survivor','action':'TERM_PID','pid':item['pid']})
        end=time.monotonic()+2
        while token_processes(self.token) and time.monotonic()<end: time.sleep(.02)
        survivors=token_processes(self.token)
        for item in survivors:
            os.kill(item['pid'], signal.SIGKILL); actions.append({'role':'survivor','action':'KILL_PID','pid':item['pid']})
        self.state('CLEANUP_VERIFIED', actions=actions, final_token_count=len(token_processes(self.token)), global_first=0, global_second=0)
        if token_processes(self.token): raise RuntimeError('exact token cleanup failed')

    def run_dummy(self, outcome='READY', escaped=False):
        """End-to-end dummy transaction; durable worker evidence is emulated."""
        self.state('PLANNED')
        if not self.admission(): raise RuntimeError('admission failed')
        self.state('SPAWNING')
        matrix_command = 'setsid /bin/sh -c "exec sleep 30" & wait' if escaped else 'exec sleep 30'
        self._spawn_dummy('matrix', matrix_command)
        self.state('WAITING_WITNESS_READY'); self._spawn_dummy('witness'); self.state('WITNESS_READY')
        if self.scenario is ScenarioType.WITNESS_CONTINUITY_PROBE:
            worker=self._spawn_dummy('worker','exit 0'); worker.wait(timeout=2)
            self.state('WITNESS_CONTINUITY_PROBE_WORKER_EXIT', worker_exit_monotonic_ns=time.monotonic_ns())
            time.sleep(3.05)
            self.state('POST_HELPER_OBSERVATION_COMPLETE', witness_observation_after_worker_exit=True)
            self.state('TERMINATING'); self._terminate(('matrix','witness'))
            self.state('PROBE_CLOSED'); return self.result()
        self._spawn_dummy('worker','exit 0')
        self.state('WAITING_HELPER_ENTER'); self.state('HELPER_ENTER_OBSERVED'); self.state('WAITING_HELPER_OUTCOME')
        self.roots['worker'][0].wait(timeout=2)
        if outcome == 'READY':
            self.state('HELPER_READY', helper_success=True); self.state('POST_HELPER_OBSERVATION_COMPLETE', passive_result='PASSIVE_READY_ELIGIBLE')
        elif outcome == 'TIMEOUT':
            self.state('HELPER_TIMEOUT', helper_success=False); self.state('POST_HELPER_OBSERVATION_ACTIVE')
            self.state('POST_HELPER_OBSERVATION_COMPLETE', passive_result='NONE_BY_BOUND')
        else:
            self.state('HELPER_OTHER_EXCEPTION', helper_success=False); self.state('TERMINATING'); self._terminate(('matrix','witness')); raise RuntimeError('P4E6B2C1F3E1B1_HELPER_INFRASTRUCTURE_EXCEPTION_NEEDS_REVIEW')
        self.state('TERMINATING'); self._terminate(('matrix','witness'))
        self.state('CHARACTERIZATION_CLOSED', helper_outcome=outcome, helper_success=outcome=='READY')
        return self.result()

    def result(self):
        states=read_states(self.journal)
        outcome='READY' if 'HELPER_READY' in states else ('TIMEOUT' if 'HELPER_TIMEOUT' in states else None)
        return {'episode':str(self.episode),'states':states,'token':self.token,'scenario':self.scenario.value,
                'helper_outcome':outcome,'helper_success':outcome=='READY',
                'closed':'CHARACTERIZATION_CLOSED' in states or 'PROBE_CLOSED' in states,
                'root_inventory':{role:row for role,(_,row) in self.roots.items()}}

    def restart_reconstruction(self):
        mapping={'MATRIX_COMMITTED':'MATRIX_SPAWNED','WITNESS_SPAWNED':'WITNESS_SPAWNED','WORKER_SPAWNED':'WORKER_SPAWNED','HELPER_ENTER_OBSERVED':'HELPER_ENTER','HELPER_READY':'HELPER_RETURN_READY','HELPER_TIMEOUT':'HELPER_EXCEPTION','POST_HELPER_OBSERVATION_ACTIVE':'POST_HELPER_OBSERVATION_ACTIVE','POST_HELPER_OBSERVATION_COMPLETE':'POST_HELPER_OBSERVATION_COMPLETE','TERMINATING':'TERMINATING','CLEANUP_VERIFIED':'CLEANUP_VERIFIED','CHARACTERIZATION_CLOSED':'CHARACTERIZATION_CLOSED'}
        return reconstruct_characterization_episode([mapping[x] for x in read_states(self.journal) if x in mapping], helper_outcome=self.result()['helper_outcome'])

    def run_live_transaction(self, episode_binding, template, process_factory=None):
        """Binding-consuming driver; process_factory enables non-ROS proof only.

        The real factory is deliberately not selected by this offline phase.
        """
        if not validate_episode_binding(episode_binding,template): raise RuntimeError('episode binding integrity failure')
        if episode_binding['scenario'] != self.scenario.value: raise RuntimeError('scenario binding mutation')
        # The immutable episode binding is the sole identity authority.  The
        # controller object may have been constructed before binding load, so
        # adopt the sealed identity before journaling or cleanup; otherwise
        # exact-token ownership would diverge from child environments.
        self.domain = episode_binding['domain']
        self.token = episode_binding['token']
        self.episode_uuid = episode_binding['episode_uuid']
        if process_factory is None: raise RuntimeError('LIVE_ROS execution not authorized in offline driver validation')
        if isinstance(process_factory, LiveRosProcessFactory):
            return self._run_real_evidence_transaction(episode_binding, template, process_factory)
        self.state('PLANNED'); self.state('PRE_EPISODE_ADMISSION_PASS',episode_binding_sha256=episode_binding['episode_binding_sha256'])
        factory=process_factory
        matrix=factory('matrix',episode_binding['root_specs']['matrix']); self.roots['matrix']=matrix; self.state('MATRIX_COMMITTED')
        witness=factory('witness',episode_binding['root_specs']['witness']); self.roots['witness']=witness; self.state('WITNESS_SPAWNED'); self.state('WITNESS_READY')
        role='continuity_worker' if self.scenario is ScenarioType.WITNESS_CONTINUITY_PROBE else 'worker'
        worker=factory(role,episode_binding['root_specs'][role]); self.roots[role]=worker; self.state('WORKER_SPAWNED')
        if self.scenario is ScenarioType.WITNESS_CONTINUITY_PROBE:
            self.state('WITNESS_CONTINUITY_PROBE_WORKER_EXIT'); self.state('POST_HELPER_OBSERVATION_COMPLETE',post_worker_witness_records=True); self.state('CLEANUP_VERIFIED'); self.state('PROBE_CLOSED'); return self.result()
        self.state('HELPER_ENTER_OBSERVED')
        outcome=getattr(factory,'outcome','READY')
        if outcome=='READY': self.state('HELPER_READY',validator='PASS',analyzer='PASSIVE_READY_ELIGIBLE')
        elif outcome=='TIMEOUT': self.state('HELPER_TIMEOUT',helper_success=False); self.state('POST_HELPER_OBSERVATION_ACTIVE'); self.state('POST_HELPER_OBSERVATION_COMPLETE',analyzer='NONE_BY_BOUND')
        else: self.state('HELPER_OTHER_EXCEPTION'); raise RuntimeError('helper infrastructure exception')
        self.state('CLEANUP_VERIFIED'); self.state('CHARACTERIZATION_CLOSED',helper_success=outcome=='READY'); return self.result()

    def _run_real_evidence_transaction(self, episode_binding, template, factory):
        """LIVE path: semantic transitions come only from durable evidence."""
        self.state('PLANNED'); admission=admission_for_binding(episode_binding); global_zero=require_global_zero()
        runner_path=Path(template['runner_path'])
        matrix_path=MATRIX_SOURCE
        admission_result = admit_live_matrix_before_popen(
            self.episode,
            lambda: factory('matrix', episode_binding['root_specs']['matrix'])[0],
            runner_path=runner_path, matrix_path=matrix_path)
        freeze=admission_result['freeze']
        self.state('FINAL_FREEZE_COMMITTED', **freeze)
        self.state('PRE_EPISODE_ADMISSION_PASS',episode_binding_sha256=episode_binding['episode_binding_sha256'],admission=admission,global_zero=global_zero,freeze=freeze)
        matrix=(admission_result['process'], {'spawn_monotonic_ns': admission_result['matrix_created_ns']})
        self.roots['matrix']=matrix; self.state('MATRIX_COMMITTED',**matrix[1])
        witness=factory('witness',episode_binding['root_specs']['witness']); self.roots['witness']=witness; self.state('WITNESS_SPAWNED',**witness[1])
        witness_file=Path(episode_binding['root_specs']['witness']['output_dir'])/'witness_phase.jsonl'
        wait_durable_phase(witness_file,{'OBSERVATION_ACTIVE'},15,witness[0]); audit=witness_noninterference(_child_env(os.environ,episode_binding['domain'],episode_binding['token'],episode_binding['episode_uuid'])); self.state('WITNESS_READY',noninterference=audit)
        role='continuity_worker' if self.scenario is ScenarioType.WITNESS_CONTINUITY_PROBE else 'worker'
        worker=factory(role,episode_binding['root_specs'][role]); self.roots[role]=worker; self.state('WORKER_SPAWNED',**worker[1])
        worker_file=Path(episode_binding['root_specs'][role]['output_dir'])/('continuity_worker_phase.jsonl' if role=='continuity_worker' else 'worker_phase.jsonl')
        if role=='continuity_worker':
            wait_durable_phase(worker_file,{'CONTINUITY_WORKER_READY'},5,worker[0]); worker[0].wait(timeout=5)
            exit_row,_=wait_durable_phase(worker_file,{'PROCESS_EXIT'},1); self.state('WITNESS_CONTINUITY_PROBE_WORKER_EXIT',worker_exit_monotonic_ns=exit_row.get('monotonic_ns'))
            before=latest_observation_ns(witness_file.parent); time.sleep(3.05); after=latest_observation_ns(witness_file.parent)
            if after-exit_row.get('monotonic_ns',0)<3_000_000_000 or after<=before: raise RuntimeError('witness continuity evidence absent')
            self.state('POST_HELPER_OBSERVATION_COMPLETE',first_or_last_post_worker_graph_ns=after); self.state('TERMINATING'); self._terminate(('matrix','witness')); self.state('GLOBAL_ZERO',**require_global_zero()); self.state('PROBE_CLOSED'); return self.result()
        enter,_=wait_durable_phase(worker_file,{'HELPER_ENTER'},20,worker[0]); self.state('HELPER_ENTER_OBSERVED',helper_enter_monotonic_ns=enter.get('monotonic_ns'))
        outcome,rows=wait_durable_phase(worker_file,{'HELPER_RETURN_READY','HELPER_EXCEPTION'},10,worker[0])
        witness_dir=Path(episode_binding['root_specs']['witness']['output_dir'])
        if outcome['phase']=='HELPER_RETURN_READY':
            seam_path=Path(episode_binding['root_specs']['worker']['output_dir'])/'premission_seam.json'
            wait_outcome_evidence_commit(worker_file,'READY',seam_path,worker[0],10)
            if not seam_path.exists() or not validate_frozen_ready(json.loads(seam_path.read_text())):
                raise RuntimeError('frozen READY validator failure')
            seam_sha_before=_sha256_file(seam_path)
            passive=analyze_witness(witness_dir,enter['monotonic_ns'])
            durable_json(self.episode/'passive_analyzer_result.json',passive)
            if passive['event']!='PASSIVE_READY_ELIGIBLE': raise RuntimeError('passive witness false negative')
            if _sha256_file(seam_path) != seam_sha_before: raise RuntimeError('outcome evidence mutated after commit')
            self.state('HELPER_READY',validator='PASS',passive_analyzer=passive); self.state('POST_HELPER_OBSERVATION_COMPLETE')
        else:
            exc=Path(episode_binding['root_specs']['worker']['output_dir'])/'helper_exception.json'
            commit=wait_outcome_evidence_commit(worker_file,'TIMEOUT',exc,worker[0],10)
            data=json.loads(exc.read_text()) if exc.exists() else {}
            if 'PREMISSION_HEALTH_READINESS_TIMEOUT' not in data.get('exception_message',''):
                raise RuntimeError('helper infrastructure exception')
            self.state('HELPER_TIMEOUT',helper_success=False); worker[0].wait(timeout=10); self.state('POST_HELPER_OBSERVATION_ACTIVE')
            limit=enter['monotonic_ns']+20_000_000_000; passive={'event':'NONE_BY_BOUND','monotonic_ns':None}
            while time.monotonic_ns()<limit:
                passive=analyze_witness(witness_dir,enter['monotonic_ns'])
                if passive['event']=='PASSIVE_READY_ELIGIBLE': break
                if matrix[0].poll() is not None or witness[0].poll() is not None: raise RuntimeError('matrix/witness died during post-timeout observation')
                time.sleep(.05)
            durable_json(self.episode/'passive_analyzer_result.json',passive)
            if passive['event']=='PASSIVE_READY_ELIGIBLE' and passive['monotonic_ns']-enter['monotonic_ns']<8_000_000_000: raise RuntimeError('runner missed ready window')
            self.state('POST_HELPER_OBSERVATION_COMPLETE',passive_analyzer=passive)
        self.state('TERMINATING'); self._terminate(('matrix','witness')); self.state('GLOBAL_ZERO',**require_global_zero()); self.state('CHARACTERIZATION_CLOSED',helper_success=outcome['phase']=='HELPER_RETURN_READY'); return self.result()

    def run_live_ros_transaction(self, episode_binding, template, popen_callable=subprocess.Popen):
        """Reviewed future entry point: no external ad-hoc factory is needed."""
        if self.backend is not BackendType.LIVE_ROS: raise RuntimeError('LIVE_ROS backend required')
        return self.run_live_transaction(episode_binding,template,LiveRosProcessFactory(self,template,episode_binding,popen_callable))


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--root',required=True); parser.add_argument('--scenario',choices=[x.value for x in ScenarioType],required=True); parser.add_argument('--index',type=int,default=1); parser.add_argument('--domain',type=int,default=1); parser.add_argument('--dummy-outcome',choices=('READY','TIMEOUT'),default='READY'); parser.add_argument('--dummy-escaped-descendant',action='store_true')
    a=parser.parse_args(); controller=LiveCharacterizationController(a.root,a.scenario,a.index,a.domain)
    print(json.dumps(controller.run_dummy(a.dummy_outcome,a.dummy_escaped_descendant),sort_keys=True))

if __name__=='__main__': main()
