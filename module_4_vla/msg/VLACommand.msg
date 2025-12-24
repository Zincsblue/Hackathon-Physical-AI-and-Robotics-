# Message definition for Vision-Language-Action commands

string command_text
string command_type  # "navigation", "manipulation", "interaction", etc.
string priority      # "low", "medium", "high"
float32 confidence_score
string[] entities    # detected entities in the command