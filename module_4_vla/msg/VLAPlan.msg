# Message definition for Vision-Language-Action plans

string plan_id
string original_command
string[] action_sequence
float32[] action_timings
string[] action_descriptions
string status  # "planning", "ready", "executing", "completed", "failed"
float32 confidence_score