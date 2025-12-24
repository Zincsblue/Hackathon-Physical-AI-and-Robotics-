# Message definition for Vision-Language-Action results

string plan_id
bool success
string message
float32 execution_time
string[] executed_actions
string[] failed_actions
string status  # "success", "partial", "failed", "timeout"