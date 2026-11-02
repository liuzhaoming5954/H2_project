# stamp is the ORIGINAL command creation time, never the relay publication time.
# frame_id must be the configured horizontal robot frame (base_footprint).
std_msgs/Header header
string source
# Fresh session returned by SetMotionEnabled; old sessions never resume motion.
string session_id
# Strictly increasing within a source/session. Repetition must not extend expiry.
uint64 sequence
geometry_msgs/Twist velocity
# Lifetime measured from header.stamp; all relays must preserve it.
# A zero validity with a zero velocity is an immediate stop.
builtin_interfaces/Duration validity
