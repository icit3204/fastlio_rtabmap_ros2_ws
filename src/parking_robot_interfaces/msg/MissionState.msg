# Mission state constants. current_waypoint_index is zero-based.
uint8 IDLE=0
uint8 VALIDATING=1
uint8 READY=2
uint8 RUNNING=3
uint8 PAUSED=4
uint8 CANCELING=5
uint8 SUCCEEDED=6
uint8 FAILED=7
uint8 BLOCKED=8
uint8 REJECTED=9

std_msgs/Header header
string mission_id
string route_id
uint8 state
uint32 current_waypoint_index
uint32 completed_waypoint_count
uint32 total_waypoint_count
float32 progress
string active_goal_uuid
string reason_code
string detail
