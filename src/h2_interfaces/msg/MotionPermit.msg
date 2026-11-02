# Short, explicit supervisor lease. It cannot itself arm the motion guard.
std_msgs/Header header
# Issuer identity, e.g. supervisor; distinct from the command source it authorizes.
string source
string session_id
uint64 sequence
bool enabled
builtin_interfaces/Duration validity
