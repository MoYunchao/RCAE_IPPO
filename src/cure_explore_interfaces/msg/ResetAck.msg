# Reset acknowledgment message
# Sent back to system_reset_server to confirm reset operations

string source            # Source identifier, e.g., "robot_1", "simulation", "cure_explore"
bool success             # Whether the reset operation succeeded
string message           # Optional status message or error description
