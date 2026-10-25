# Reset command message for system reset coordination
# Used to control lifecycle of nodes across different domains

# Command types:
# 0 = SHUTDOWN: Deactivate and cleanup nodes
# 1 = STARTUP: Configure and activate nodes
# 2 = RESTART: Full restart (shutdown + startup)

uint8 command            # The command to execute (0=SHUTDOWN, 1=STARTUP, 2=RESTART)
string[] node_names      # Specific nodes to operate on (empty = all managed nodes)
