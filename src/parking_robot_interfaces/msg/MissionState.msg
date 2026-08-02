# Mission state constants. current_waypoint_index is zero-based.
uint8 IDLE=0
uint8 RECEIVED=1
uint8 VALIDATING=2
uint8 PLANNING=3
uint8 NAVIGATING=4
uint8 PAUSED=5
uint8 CANCELLING=6
uint8 CANCELLED=7
uint8 SUCCEEDED=8
uint8 TEMPORARILY_BLOCKED=9
uint8 BLOCKED=10
uint8 FAILED=11
uint8 HELP_REQUIRED=12

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
