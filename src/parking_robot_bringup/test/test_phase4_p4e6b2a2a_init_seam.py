import pytest
from parking_robot_bringup.phase4_p4e6b2a2a_init_seam import run


@pytest.mark.parametrize("ros,expected",[
 (["--ros-args","-r","__node:=node_only"],"node_only"),
 (["--ros-args","-r","__ns:=/qualified"],"p4e6b2a2a_unremapped"),
 (["--ros-args","-r","__node:=both","-r","__ns:=/qualified"],"both"),
 (["--ros-args","-r","/unused:=/remapped"],"p4e6b2a2a_unremapped"),
 (["--ros-args","-r","__node:=multi","-r","__ns:=/q","-r","/a:=/b"],"multi")])
def test_real_rclpy_init_variants(tmp_path,ros,expected):
 raw=["--case-id","B-H01","--output-dir",str(tmp_path),"--execute-authorized-case",*ros]
 result=run(raw);assert result["pass"] and result["effective_node_name"]==expected
 assert result["campaign_object_created"] is False
 assert all(result[x]==0 for x in ("route_mission","mission_start","gate_arm","health_injection"))
 assert not any(x in result["application_argv"] for x in ros)

def test_exact_attempt_one_raw_vector_real_init(tmp_path):
 raw=["--case-id","B-H01","--output-dir",str(tmp_path),"--execute-authorized-case",
      "--ros-args","-r","__node:=phase4_p4e6b_health_failure_runner"]
 result=run(raw);assert result["effective_node_name"]=="phase4_p4e6b_health_failure_runner"

@pytest.mark.parametrize("bad",["--typo","--case-idd"])
def test_unknown_application_flag_still_rejected(tmp_path,bad):
 with pytest.raises(SystemExit):run(["--case-id","B-H01","--output-dir",str(tmp_path),
   "--execute-authorized-case",bad,"--ros-args","-r","__node:=x"])
