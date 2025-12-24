# Prompt Engineering Examples for LLM-Based Task Planning

## Overview

Effective prompt engineering is crucial for generating high-quality task plans from LLMs. This document provides examples of well-crafted prompts that guide LLMs toward generating structured, safe, and executable plans for VLA systems.

## Basic Task Planning Prompts

### 1. Simple Command-to-Action Mapping

```
You are a robot task planner. Convert the following command into a sequence of specific actions.

Command: "Bring me the red cup from the table"

Available Actions: navigate_to_location, detect_object, approach_object, grasp_object, transport_object, place_object

Output the plan as a JSON list of actions with parameters and reasoning:

[
  {
    "action": "detect_object",
    "parameters": {"object_type": "red cup", "search_location": "table"},
    "reasoning": "First, locate the red cup on the table"
  },
  {
    "action": "navigate_to_location",
    "parameters": {"location": "table"},
    "reasoning": "Move to the table where the cup was detected"
  }
]
```

### 2. Context-Aware Planning

```
You are a robot task planner. Plan actions considering the current environmental state.

Command: "Move the book to the shelf"
Current Context:
- Robot position: living room
- Book location: coffee table in living room
- Shelf location: bookshelf in study room
- Obstacles: couch, armchair

Available Actions: navigate_to_location, detect_object, approach_object, grasp_object, transport_object, place_object, avoid_obstacle

Generate a JSON plan with:
- Action sequence
- Reasoning for each action
- Safety considerations
- Estimated duration for each action
```

### 3. Safety-Conscious Planning

```
You are a safety-conscious robot task planner. Generate a plan that prioritizes safety.

Command: "Get the glass from the table"
Constraints:
- Avoid fragile objects
- Maintain safe distances from humans
- Check for obstacles

Available Actions: detect_object, approach_object, grasp_object, navigate_safely, transport_safely

Include in your response:
- Action sequence with safety checks
- Risk assessment for each action
- Fallback procedures if something goes wrong

Output in structured JSON format.
```

## Advanced Prompting Techniques

### 4. Role-Based Prompting

```
As an expert robot task planner with 10 years of experience, analyze the following command:

Role: Senior Robot Task Planner
Expertise: Navigation, manipulation, safety protocols, human-robot interaction
Current Command: "Set the table for dinner"

Environmental Context:
- Kitchen: available
- Dining room: available
- Objects: plates, cups, utensils
- Table dimensions: 1.2m x 0.8m

Plan Requirements:
- Consider human ergonomics
- Efficient path planning
- Safety during manipulation

Please think step-by-step and provide your final plan in JSON format.
```

### 5. Chain-of-Thought Reasoning

```
Let's think step-by-step to plan the following task:

Command: "Clean the living room"

Step 1: Analyze the command
- What needs to be cleaned?
- What objects are in the way?
- What cleaning tools are needed?

Step 2: Assess the environment
- Current state of the living room
- Object locations
- Robot capabilities

Step 3: Plan the sequence
- Which objects to move first?
- What cleaning actions are needed?
- How to avoid re-contaminating cleaned areas?

Step 4: Consider safety
- Safe object handling
- Path safety
- Human safety

Now generate the detailed action plan in JSON format with reasoning for each step.
```

### 6. Multi-Step Decomposition

```
Decompose the complex command into hierarchical subtasks:

Command: "Prepare dinner for two people"

Level 1 (High-level tasks):
- Set table
- Prepare food
- Serve food

For each Level 1 task, provide 2-3 Level 2 subtasks:

Set table:
- Place plates
- Place utensils
- Place glasses

Continue this hierarchical decomposition down to atomic actions (Level 4), then provide the complete execution sequence.

Output format:
{
  "hierarchy": {...},
  "execution_sequence": [...]
}
```

## Domain-Specific Prompting

### 7. Household Tasks

```
You are a household robot assistant. Plan tasks considering typical home environments.

Command: {{command}}

Home Context:
- Furniture: {{furniture}}
- Common obstacles: {{obstacles}}
- Safety requirements: {{safety_requirements}}

Special considerations for home environments:
- Don't disturb sleeping humans
- Handle fragile items carefully
- Return items to proper locations
- Consider household routines

Generate executable plan with safety checks and human-awareness.
```

### 8. Industrial Tasks

```
You are an industrial robot planner. Focus on efficiency and precision.

Command: {{command}}

Industrial Context:
- Factory layout: {{layout}}
- Safety protocols: {{protocols}}
- Precision requirements: {{precision}}
- Quality standards: {{standards}}

Requirements:
- Optimize for speed and efficiency
- Follow strict safety protocols
- Meet quality standards
- Report anomalies

Generate optimized plan with quality checks.
```

## Prompt Templates

### 9. Flexible Prompt Template

```
# Robot Task Planning Prompt Template

## Role
You are a {{role}} robot task planner.

## Context
- Command: {{command}}
- Environment: {{environment}}
- Current state: {{state}}
- Constraints: {{constraints}}
- Available actions: {{actions}}

## Requirements
- {{requirement1}}
- {{requirement2}}
- {{requirement3}}

## Output Format
Please respond in the following JSON format:
{
  "action_sequence": [
    {
      "action": "action_name",
      "parameters": {"param": "value"},
      "reasoning": "why this action",
      "estimated_duration": seconds
    }
  ],
  "reasoning": "overall plan reasoning",
  "safety_considerations": ["consideration1", "consideration2"],
  "estimated_total_duration": seconds,
  "confidence": 0.0-1.0
}

## Additional Instructions
- {{instruction1}}
- {{instruction2}}
```

### 10. Safety-First Template

```
# Safety-First Robot Planning Prompt

You are a safety-critical robot task planner. Your primary objective is to ensure safe operation above all else.

COMMAND: {{command}}

SAFETY CONSTRAINTS:
- Human safety: {{human_safety}}
- Object safety: {{object_safety}}
- Environmental safety: {{environment_safety}}
- Robot safety: {{robot_safety}}

ENVIRONMENTAL CONTEXT: {{context}}

For each action, consider:
1. Could this action harm a human?
2. Could this action damage an object?
3. Could this action cause an environmental hazard?
4. Could this action damage the robot?

Generate a plan that explicitly addresses each safety concern. If the command cannot be executed safely, explain why and suggest alternatives.

Output format: JSON with safety checks for each action.
```

## Error Handling Prompts

### 11. Failure Mode Planning

```
Plan for potential failure modes:

Command: "Pick up the glass"

Possible Failures:
- Glass not found
- Grasp fails
- Glass breaks during transport
- Path is blocked

For each failure mode, include:
- Detection method
- Recovery action
- Escalation procedure

Plan with failure recovery built in:

{
  "primary_sequence": [...],
  "failure_handlers": {
    "glass_not_found": {
      "detection": "object_detection_timeout",
      "recovery": "search_alternative_location",
      "escalation": "request_human_assistance"
    }
  }
}
```

## Examples of Effective Prompts

### 12. Real-World Example: Object Retrieval

```
You are a service robot assistant. Plan to retrieve an object for a user.

USER COMMAND: "Please bring me my reading glasses."

ENVIRONMENT CONTEXT:
- User location: living room couch
- Known possible locations for glasses: bedroom nightstand, living room coffee table, kitchen counter
- Robot current location: charging dock near kitchen

ROBOT CAPABILITIES:
- Navigation precision: ±5cm
- Object detection range: 3m
- Maximum carry weight: 0.5kg
- Manipulation precision: ±2cm

REQUIREMENTS:
- Prioritize most likely locations first
- Announce object detection to user
- Use safest path avoiding obstacles
- Handle fragile objects carefully

THINK STEP-BY-STEP:
1. Where are reading glasses most commonly found?
2. What's the most efficient search pattern?
3. How to verify these are the user's glasses?
4. How to transport safely?

OUTPUT: Detailed JSON plan with search strategy, verification steps, and safe transport.
```

### 13. Real-World Example: Navigation Task

```
You are a navigation-focused robot planner.

COMMAND: "Go to the main entrance and wait there."

BUILDING CONTEXT:
- Building type: office building
- Floors: 5 total
- Current location: 3rd floor, room 301
- Main entrance: ground floor, near reception
- Elevator available
- Stairs available

SAFETY REQUIREMENTS:
- Yield to pedestrians
- Stop for elevator doors
- Announce presence at intersections
- Avoid construction areas

NAVIGATION REQUIREMENTS:
- Take most efficient route
- Use elevator for multi-floor movement
- Follow traffic patterns
- Wait in designated area

Generate turn-by-turn navigation plan with safety checks and communication points.
```

## Best Practices for Prompt Engineering

### 14. Prompt Structure Guidelines

1. **Clear Role Definition**: Always specify the role of the LLM
2. **Context First**: Provide environmental context before the command
3. **Action List**: Include available actions as a reference
4. **Format Requirements**: Specify output format clearly
5. **Safety Priorities**: Include safety requirements early
6. **Step-by-Step**: Use CoT for complex tasks
7. **Examples**: Include few-shot examples for consistency

### 15. Validation Prompts

```
Validate the following robot plan for safety and feasibility:

PLAN: {{generated_plan}}

VALIDATION CRITERIA:
- Does each action reference available robot capabilities?
- Are safety considerations addressed?
- Is the sequence logically ordered?
- Are resource constraints respected?
- Are failure recovery procedures included?

Rate each criterion 1-5 and provide specific feedback for improvements.
```

## Testing and Optimization

### 16. A/B Testing Prompts

Use different prompt structures to compare effectiveness:

**Prompt A**: Direct command → action mapping
**Prompt B**: Step-by-step reasoning required
**Prompt C**: Role-based with expertise context

Evaluate based on:
- Plan quality (completeness, safety, efficiency)
- Execution success rate
- Human satisfaction
- Safety incident rate

These examples provide a foundation for effective prompt engineering in VLA systems. The key is to balance specificity with flexibility, ensuring the LLM generates actionable plans while considering safety and environmental constraints.