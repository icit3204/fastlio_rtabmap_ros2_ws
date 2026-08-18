from action_msgs.msg import GoalStatus
from parking_robot_interfaces.msg import MissionState
from parking_robot_bringup.phase4_p4e3a_mission_cancel_runner import adjudicate_cancel_stop


def good(**kw):
    layers={n:[(100,True),(1_010_000_000+i,False)] for i,n in enumerate(("raw","safe","gate","mock","fake"))}
    args=dict(layers=layers,cancel_ns=1_000_000_000,action_status=GoalStatus.STATUS_CANCELED,
              mission_state=MissionState.CANCELLED,observation_end_ns=3_200_000_000,
              post_cancel_translation_m=0.001)
    args.update(kw); return adjudicate_cancel_stop(**args)


def test_cancel_contract_passes(): assert good()["pass"]
def test_wrong_action_terminal_rejected(): assert not good(action_status=GoalStatus.STATUS_ABORTED)["pass"]
def test_wrong_mission_terminal_rejected(): assert not good(mission_state=MissionState.FAILED)["pass"]
def test_later_waypoint_rejected(): assert not good(later_waypoint_dispatch=True)["pass"]
def test_short_stationary_window_rejected(): assert not good(observation_end_ns=2_000_000_000)["pass"]
def test_translation_limit_rejected(): assert not good(post_cancel_translation_m=0.02001)["pass"]


def test_later_nonzero_is_not_redefined_as_terminal_motion():
    layers={n:[(100,True),(1_010_000_000,False)] for n in ("raw","safe","gate","mock","fake")}
    layers["raw"].append((1_020_000_000,True))
    result=good(layers=layers)
    assert not result["pass"]
    assert result["layers"]["raw"]["cancellation_terminal_zero_ns"]==1_010_000_000
    assert result["layers"]["raw"]["later_nonzero_violation"]


def test_missing_layer_zero_rejected():
    layers={n:[(100,True),(1_010_000_000,False)] for n in ("raw","safe","gate","mock","fake")}
    layers["safe"]=[(100,True)]
    assert not good(layers=layers)["pass"]
