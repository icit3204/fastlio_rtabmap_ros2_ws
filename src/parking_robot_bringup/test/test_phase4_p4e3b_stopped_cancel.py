from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState
from parking_robot_bringup.phase4_p4e3b_stopped_cancel_runner import adjudicate_pre_stopped_user_cancel

CANCEL=900_000_000
def good(**kw):
    layers={"raw":[(850_000_000,True),(950_000_000,False)]}
    layers.update({n:[(300_000_000,True),(350_000_000,False),(850_000_000,False)] for n in ("safe","gate","mock","fake")})
    args=dict(layers=layers,cancel_ns=CANCEL,stop_confirmation_ns=100_000_000,full_downstream_zero_ns=350_000_000,
              action_status=GoalStatus.STATUS_CANCELED,mission_state=MissionState.CANCELLED,
              observation_end_ns=3_500_000_000,post_cancel_translation_m=.001)
    args.update(kw); return adjudicate_pre_stopped_user_cancel(**args)

def test_correct_pre_stopped_cancel(): assert good()["pass"]
def test_later_nonzero_rejected():
    raw={"raw":[(850_000_000,True),(950_000_000,False)]}; raw.update({n:[(300_000_000,True),(350_000_000,False),(850_000_000,False),(1_000_000_000,True),(1_100_000_000,False)] for n in ("safe","gate","mock","fake")})
    assert not good(layers=raw)["pass"]
def test_wrong_action_rejected(): assert not good(action_status=GoalStatus.STATUS_ABORTED)["pass"]
def test_wrong_mission_rejected(): assert not good(mission_state=MissionState.FAILED)["pass"]
def test_second_uuid_rejected(): assert not good(uuid_count=2)["pass"]
def test_waypoint_one_rejected(): assert not good(waypoint_one_dispatched=True)["pass"]
def test_short_stationary_rejected(): assert not good(observation_end_ns=3_399_999_999)["pass"]
def test_translation_rejected(): assert not good(post_cancel_translation_m=.02001)["pass"]
def test_cancel_at_threshold_rejected(): assert not good(cancel_ns=1_100_000_000)["pass"]
def test_safe_nonzero_at_cancel_rejected():
    layers={"raw":[(850_000_000,True)]}; layers.update({n:[(350_000_000,False),(850_000_000,False)] for n in ("safe","gate","mock","fake")}); layers["safe"].append((880_000_000,True))
    assert not good(layers=layers)["pass"]
