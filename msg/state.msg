# Gripper open (0) or closed (1)
int64 gripper_state 

# Left Gripper open (0) or closed (1)
int64 left_gripper_state 

# Initial stack (0) or final stack (1)
int64 stack



# New block is one that you have just moved "to", old block is a block that you have moved "over" another block

# Gripper at a "new" block (1) or "old" block (0)
int64 to_block

# Left gripper at a "new" block (1) or "old" block (0)
int64 left_to_block

symbolicblock[] symbolic_block_positions

blockposition[] block_positions
