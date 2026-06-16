std_msgs/Header header
string agent_name
uint8 agent_id
yunlink_msgs/CommandMeta meta

uint8 COMMAND_UNKNOWN=0
uint8 COMMAND_TAKEOFF=1
uint8 COMMAND_LAND=2
uint8 COMMAND_RETURN=3
uint8 COMMAND_GOTO=4
uint8 COMMAND_VELOCITY_SETPOINT=5
uint8 command_kind

uint8 IDLE=0
uint8 ACCEPTED=1
uint8 RUNNING=2
uint8 WAITING_PHYSICAL_STATE=3
uint8 SUCCEEDED=4
uint8 FAILED=5
uint8 CANCELLED=6
uint8 TIMEOUT=7
uint8 execution_state

uint8 progress_percent
bool active
bool terminal
bool success
uint16 result_code
string detail

uint8 control_state
uint8 px4_landed_state
bool ready_for_takeoff
bool ready_for_land
string busy_reason
