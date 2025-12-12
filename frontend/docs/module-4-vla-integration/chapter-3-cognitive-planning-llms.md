# Chapter 3: Cognitive Planning with LLMs

## Introduction

Cognitive planning with Large Language Models (LLMs) represents a revolutionary approach to robot task execution, where natural language commands are transformed into detailed, executable action sequences. This chapter explores how LLMs can serve as cognitive engines that understand high-level goals and decompose them into step-by-step robotic actions.

The integration of cognitive planning with LLMs enables robots to perform complex, multi-step tasks by leveraging the LLM's ability to reason about goals, constraints, and action sequences. This approach moves beyond simple command mapping to enable sophisticated task planning and execution that adapts to changing circumstances and environmental conditions.

## Cognitive Planning Concepts

### Understanding Cognitive Planning

Cognitive planning in robotics refers to the process of converting high-level goals or natural language commands into executable action sequences. It involves:

1. **Goal Analysis**: Understanding the desired end state or outcome
2. **Action Selection**: Choosing appropriate actions to achieve the goal
3. **Sequence Planning**: Ordering actions in a logical sequence
4. **Constraint Handling**: Managing environmental and robot-specific constraints
5. **Execution Monitoring**: Tracking progress and adapting to changes

### LLM-Based Planning vs Traditional Approaches

Traditional robotic planning typically involves:
- Predefined action sequences
- Deterministic state machines
- Rule-based systems
- Limited adaptability to novel situations

LLM-based cognitive planning offers:
- Natural language interface
- Flexible, context-aware planning
- Ability to handle ambiguous or complex commands
- Learning from examples and experience
- Adaptation to novel situations

### Planning Architecture

The cognitive planning architecture with LLMs typically includes:

- **Input Processing**: Natural language understanding and goal parsing
- **Context Management**: Maintaining relevant environmental and task state
- **Plan Generation**: Creating action sequences using LLM reasoning
- **Validation Layer**: Ensuring plans are executable and safe
- **Execution Interface**: Converting plans to robot commands
- **Feedback Integration**: Incorporating execution results into planning

## Prompt Engineering Techniques for Cognitive Planning

### Structured Prompting

Structured prompting is essential for reliable LLM-based planning. The following patterns have proven effective:

```python
def create_planning_prompt(goal, environment, robot_capabilities):
    """
    Create a structured prompt for cognitive planning
    """
    prompt = f"""
    You are an AI planning agent for a humanoid robot. Generate a step-by-step plan to achieve the given goal.

    Environment: {environment}
    Robot Capabilities: {robot_capabilities}
    Goal: {goal}

    Plan Format:
    1. [Action] - Description of action and purpose
    2. [Action] - Description of action and purpose
    3. [Action] - Description of action and purpose

    Requirements:
    - Each action must be executable by the robot
    - Consider environmental constraints
    - Ensure logical sequence of actions
    - Include error handling where appropriate
    - Validate against robot capabilities

    Plan:
    """
    return prompt
```

### Few-Shot Learning Examples

Providing examples helps LLMs understand the expected format and complexity:

```python
def create_few_shot_prompt():
    """
    Create a prompt with few-shot examples for better planning
    """
    prompt = """
    Plan robotic tasks using the following examples:

    Example 1:
    Goal: "Get a glass of water from the kitchen"
    Plan:
    1. Navigate to kitchen - Move to the kitchen area
    2. Locate water source - Find the sink or water dispenser
    3. Locate glass - Find a clean glass in the cupboard
    4. Pick up glass - Grasp the glass with robotic hand
    5. Move to water source - Navigate to sink
    6. Fill glass with water - Position glass under faucet
    7. Navigate back - Return to original location

    Example 2:
    Goal: "Organize books on the shelf"
    Plan:
    1. Scan bookshelf - Identify current book arrangement
    2. Identify misplaced books - Find books not in correct position
    3. Plan reorganization - Determine optimal arrangement
    4. Pick up first book - Grasp the first book to move
    5. Place book in correct position - Move book to designated spot
    6. Repeat for remaining books - Continue until all books organized

    Now plan this task:
    Goal: "Set the table for dinner"
    Plan:
    """
    return prompt
```

### Chain-of-Thought Reasoning

Encouraging LLMs to explain their reasoning before generating actions:

```python
def create_chain_of_thought_prompt(goal, environment):
    """
    Create a prompt that encourages chain-of-thought reasoning
    """
    prompt = f"""
    You are planning robotic actions. First think through the requirements, then provide the action plan.

    Goal: {goal}
    Environment: {environment}

    Step-by-step thinking:
    1. What is the end goal to achieve?
    2. What objects or locations are involved?
    3. What capabilities does the robot need?
    4. What sequence of actions would achieve the goal?
    5. What constraints or obstacles might exist?

    Action Plan:
    - Action 1: [description]
    - Action 2: [description]
    - Action 3: [description]
    """
    return prompt
```

### Context-Aware Planning

Incorporating environmental context into planning:

```python
def create_context_aware_prompt(goal, current_state, environment_map):
    """
    Create a prompt that incorporates current context
    """
    prompt = f"""
    Plan robotic actions considering current state and environment.

    Current Robot State: {current_state}
    Environment Map: {environment_map}
    Goal: {goal}

    Considerations:
    - Current location and orientation
    - Available objects and their positions
    - Obstacles and navigable paths
    - Robot's current tools or held objects

    Step-by-step Plan:
    1. [Action] - [Reasoning for action]
    2. [Action] - [Reasoning for action]
    3. [Action] - [Reasoning for action]

    Safety Checks:
    - Verify action feasibility
    - Check for environmental hazards
    - Confirm robot capability for each action
    """
    return prompt
```

## Planning Algorithms for Task Decomposition

### Hierarchical Task Networks (HTNs)

HTNs decompose complex tasks into smaller, manageable subtasks:

```python
class HierarchicalTaskNetwork:
    def __init__(self):
        self.primitive_actions = {
            'move_to': self.move_to,
            'pick_up': self.pick_up,
            'place': self.place,
            'open': self.open_container,
            'close': self.close_container,
            'speak': self.speak
        }

    def decompose_task(self, task, context):
        """
        Decompose high-level task into primitive actions
        """
        if task == 'prepare_coffee':
            return [
                {'action': 'move_to', 'params': {'location': 'kitchen'}},
                {'action': 'locate', 'params': {'object': 'coffee_machine'}},
                {'action': 'locate', 'params': {'object': 'coffee_cup'}},
                {'action': 'pick_up', 'params': {'object': 'coffee_cup'}},
                {'action': 'move_to', 'params': {'location': 'coffee_machine'}},
                {'action': 'operate', 'params': {'object': 'coffee_machine', 'action': 'brew'}},
                {'action': 'wait', 'params': {'duration': 60}},  # Wait 60 seconds
                {'action': 'move_to', 'params': {'location': 'table'}}
            ]
        elif task == 'set_table':
            return [
                {'action': 'move_to', 'params': {'location': 'dining_area'}},
                {'action': 'count_seats', 'params': {}},
                {'action': 'calculate_items_needed', 'params': {}},
                {'action': 'locate', 'params': {'object': 'plates'}},
                # ... continue with detailed steps
            ]
        else:
            # For unknown tasks, use LLM to decompose
            return self.llm_decompose_task(task, context)

    def llm_decompose_task(self, task, context):
        """
        Use LLM to decompose unfamiliar tasks
        """
        prompt = f"""
        Decompose the following task into primitive robotic actions:
        Task: {task}
        Context: {context}

        Return a list of primitive actions with parameters.
        """
        # Implementation would call LLM here
        pass
```

### Partial-Order Planning

Allowing concurrent execution of independent actions:

```python
class PartialOrderPlanner:
    def __init__(self):
        self.action_dependencies = {
            'pick_up': ['locate_object'],
            'place': ['pick_up'],
            'move_to': [],
            'open': ['move_to'],
            'close': ['open']
        }

    def create_partial_order_plan(self, goal):
        """
        Create a plan where independent actions can run concurrently
        """
        # Generate all required actions
        all_actions = self.generate_actions(goal)

        # Determine dependencies
        dependencies = self.analyze_dependencies(all_actions)

        # Create execution order respecting dependencies
        execution_plan = self.topological_sort(dependencies)

        # Group independent actions for concurrent execution
        concurrent_plan = self.group_concurrent_actions(execution_plan)

        return concurrent_plan

    def group_concurrent_actions(self, execution_plan):
        """
        Group actions that can be executed concurrently
        """
        concurrent_groups = []
        current_group = []

        for action in execution_plan:
            # Check if action conflicts with any in current group
            conflicts = any(self.actions_conflict(action, existing)
                          for existing in current_group)

            if not conflicts:
                current_group.append(action)
            else:
                if current_group:
                    concurrent_groups.append(current_group)
                current_group = [action]

        if current_group:
            concurrent_groups.append(current_group)

        return concurrent_groups

    def actions_conflict(self, action1, action2):
        """
        Determine if two actions conflict and cannot run concurrently
        """
        # Check for resource conflicts
        # Check for spatial conflicts
        # Check for temporal conflicts
        return False  # Simplified for example
```

### Goal-Regression Planning

Planning backwards from the goal state:

```python
class GoalRegressionPlanner:
    def __init__(self):
        self.action_effects = {
            'move_to(location)': ['at_location(location)'],
            'pick_up(object)': ['holding(object)', 'not_at_location(object)'],
            'place(object, location)': ['not_holding(object)', 'at_location(object, location)'],
            'open(container)': ['container_open(container)'],
            'close(container)': ['container_closed(container)']
        }

    def plan_backward(self, goal_state, current_state):
        """
        Plan backwards from goal state to current state
        """
        plan = []
        subgoals = [goal_state]

        while subgoals:
            goal = subgoals.pop(0)

            if self.state_achieved(goal, current_state):
                continue

            # Find action that achieves this goal
            action = self.find_action_for_goal(goal)

            if action:
                # Add preconditions as new subgoals
                preconditions = self.get_action_preconditions(action)
                subgoals.extend(preconditions)

                # Add action to plan
                plan.insert(0, action)

        return plan

    def find_action_for_goal(self, goal):
        """
        Find an action that can achieve the given goal
        """
        # Implementation would search for appropriate action
        pass
```

## Examples of Natural Language to ROS 2 Conversion

### Basic Command Conversion

```python
import json
from openai import OpenAI
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class NaturalLanguageToROSConverter:
    def __init__(self):
        self.client = OpenAI()  # Initialize OpenAI client
        self.ros_node = None

    def convert_command(self, natural_language_command):
        """
        Convert natural language command to ROS 2 action sequence
        """
        prompt = f"""
        Convert this natural language command to a sequence of ROS 2 actions.

        Command: "{natural_language_command}"

        Available ROS 2 Actions:
        - MoveTo: Move robot to a specific location
        - PickUp: Pick up an object
        - Place: Place an object at a location
        - Open: Open a container
        - Close: Close a container
        - Speak: Make the robot speak

        Return the action sequence in JSON format:
        {{
            "actions": [
                {{
                    "type": "MoveTo",
                    "parameters": {{"x": 1.0, "y": 2.0, "theta": 0.0}},
                    "description": "Move to location"
                }}
            ]
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)
            return result["actions"]
        except Exception as e:
            print(f"Conversion error: {e}")
            return []

    def execute_action_sequence(self, action_sequence):
        """
        Execute the converted ROS 2 action sequence
        """
        for action in action_sequence:
            self.execute_single_action(action)

    def execute_single_action(self, action):
        """
        Execute a single ROS 2 action
        """
        action_type = action["type"]

        if action_type == "MoveTo":
            self.execute_move_to(action["parameters"])
        elif action_type == "PickUp":
            self.execute_pick_up(action["parameters"])
        elif action_type == "Place":
            self.execute_place(action["parameters"])
        elif action_type == "Open":
            self.execute_open(action["parameters"])
        elif action_type == "Close":
            self.execute_close(action["parameters"])
        elif action_type == "Speak":
            self.execute_speak(action["parameters"])
        else:
            print(f"Unknown action type: {action_type}")

    def execute_move_to(self, params):
        """
        Execute MoveTo action
        """
        # Implementation for MoveTo action
        print(f"Moving to position: {params}")

    def execute_pick_up(self, params):
        """
        Execute PickUp action
        """
        # Implementation for PickUp action
        print(f"Picking up object: {params}")

    # ... other action execution methods
```

### Complex Multi-Step Conversion

```python
class ComplexTaskConverter:
    def __init__(self):
        self.client = OpenAI()
        self.environment_context = {
            "locations": ["kitchen", "living_room", "bedroom", "dining_area"],
            "objects": ["cup", "book", "plate", "bottle"],
            "robot_capabilities": ["move", "grasp", "speak", "navigate"]
        }

    def convert_complex_command(self, command, context=None):
        """
        Convert complex multi-step commands with environmental context
        """
        if context is None:
            context = self.environment_context

        prompt = f"""
        Convert this complex command to a detailed ROS 2 action sequence.

        Command: "{command}"
        Environmental Context: {json.dumps(context, indent=2)}

        Considerations:
        1. Break down into executable steps
        2. Account for environmental constraints
        3. Validate against robot capabilities
        4. Include error handling where appropriate

        Action Sequence Format:
        {{
            "actions": [
                {{
                    "id": "action_1",
                    "type": "MoveTo",
                    "parameters": {{"target_location": "kitchen"}},
                    "description": "Move to kitchen to begin task",
                    "preconditions": [],
                    "postconditions": ["at_location_kitchen"],
                    "timeout": 30.0
                }}
            ],
            "metadata": {{
                "estimated_duration": 120.0,
                "complexity": "high",
                "dependencies": []
            }}
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)
            return result
        except Exception as e:
            print(f"Complex conversion error: {e}")
            return {"actions": [], "metadata": {}}

    def validate_action_sequence(self, action_sequence):
        """
        Validate the action sequence before execution
        """
        actions = action_sequence.get("actions", [])
        validation_results = []

        for i, action in enumerate(actions):
            # Validate action type
            if "type" not in action:
                validation_results.append({
                    "index": i,
                    "error": "Missing action type",
                    "valid": False
                })
                continue

            # Validate parameters
            if "parameters" not in action:
                validation_results.append({
                    "index": i,
                    "error": "Missing action parameters",
                    "valid": False
                })
                continue

            # Check if action is supported by robot
            action_type = action["type"]
            if action_type not in self.environment_context["robot_capabilities"]:
                validation_results.append({
                    "index": i,
                    "error": f"Robot cannot perform {action_type}",
                    "valid": False
                })
                continue

            validation_results.append({
                "index": i,
                "valid": True
            })

        return validation_results
```

### Safety-First Conversion

```python
class SafeCommandConverter:
    def __init__(self):
        self.client = OpenAI()
        self.safety_constraints = {
            "forbidden_actions": ["jump", "run_fast", "lift_heavy"],
            "weight_limits": {"max_pickup_weight": 5.0},  # 5 kg
            "distance_limits": {"max_navigation_distance": 20.0},  # 20 meters
            "speed_limits": {"max_linear_speed": 1.0}  # 1 m/s
        }

    def convert_with_safety_check(self, command):
        """
        Convert command with built-in safety validation
        """
        prompt = f"""
        Convert this command to ROS 2 actions with safety considerations.

        Command: "{command}"
        Safety Constraints: {json.dumps(self.safety_constraints, indent=2)}

        Requirements:
        1. Check if the requested action is safe
        2. Verify it's within robot capabilities
        3. Include safety validation steps
        4. Add error recovery procedures

        Safe Action Sequence:
        {{
            "actions": [
                {{
                    "type": "ValidateAction",
                    "parameters": {{"action": "MoveTo", "target": "location"}},
                    "safety_check": true
                }},
                {{
                    "type": "MoveTo",
                    "parameters": {{"target_location": "location"}},
                    "safety_check": true
                }}
            ],
            "safety_validation": {{
                "action_safe": true,
                "risk_level": "low",
                "safety_score": 0.95
            }}
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)

            # Perform additional safety validation
            safety_ok = self.validate_safety(result)

            if safety_ok:
                return result
            else:
                return self.generate_safe_alternative(command)
        except Exception as e:
            print(f"Safe conversion error: {e}")
            return {"actions": [], "safety_validation": {"action_safe": false}}

    def validate_safety(self, action_sequence):
        """
        Perform additional safety validation
        """
        # Implementation of safety validation
        return True

    def generate_safe_alternative(self, command):
        """
        Generate a safe alternative to a potentially unsafe command
        """
        prompt = f"""
        The original command may be unsafe. Generate a safe alternative.

        Original Command: "{command}"
        Safety Constraints: {json.dumps(self.safety_constraints, indent=2)}

        Safe Alternative Action Sequence in JSON format.
        """
        # Implementation would call LLM for safe alternative
        pass
```

## LLM Planning Workflow Example

### Complete End-to-End Workflow

```python
class LLMPlanningWorkflow:
    def __init__(self):
        self.converter = NaturalLanguageToROSConverter()
        self.validator = SafeCommandConverter()
        self.planner = HierarchicalTaskNetwork()
        self.executor = None  # ROS 2 node for execution

    def execute_planning_workflow(self, command, context=None):
        """
        Complete workflow from natural language to robot execution
        """
        print(f"Processing command: {command}")

        # Step 1: Parse and understand the command
        parsed_command = self.parse_command(command)
        print(f"Parsed command: {parsed_command}")

        # Step 2: Generate action plan using LLM
        action_plan = self.converter.convert_command(command)
        print(f"Generated action plan: {action_plan}")

        # Step 3: Validate the plan for safety and feasibility
        validation_results = self.validator.validate_safety({"actions": action_plan})
        if not validation_results.get("safety_validation", {}).get("action_safe", False):
            print("Action plan failed safety validation")
            return False

        # Step 4: Decompose complex tasks if needed
        detailed_plan = self.planner.decompose_task(command, context or {})
        print(f"Detailed plan: {detailed_plan}")

        # Step 5: Execute the plan
        success = self.execute_plan(detailed_plan)

        return success

    def parse_command(self, command):
        """
        Parse the natural language command
        """
        # Implementation for command parsing
        return {"original": command, "intent": "unknown", "entities": []}

    def execute_plan(self, plan):
        """
        Execute the detailed action plan
        """
        try:
            for action in plan:
                print(f"Executing: {action}")
                # Execute each action with error handling
                success = self.execute_single_action_with_retry(action)
                if not success:
                    print(f"Action failed: {action}")
                    return False

            print("Plan executed successfully")
            return True
        except Exception as e:
            print(f"Plan execution failed: {e}")
            return False

    def execute_single_action_with_retry(self, action, max_retries=3):
        """
        Execute a single action with retry logic
        """
        for attempt in range(max_retries):
            try:
                # Execute action
                result = self.execute_action(action)
                if result:
                    return True
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                if attempt == max_retries - 1:
                    return False
                # Wait before retry
                time.sleep(1)

        return False

# Example usage of the complete workflow
def main():
    workflow = LLMPlanningWorkflow()

    # Example commands
    commands = [
        "Go to the kitchen and bring me a glass of water",
        "Move the book from the table to the shelf",
        "Navigate to the living room and turn on the light"
    ]

    for cmd in commands:
        print(f"\n--- Processing Command: {cmd} ---")
        success = workflow.execute_planning_workflow(cmd)
        print(f"Command {'succeeded' if success else 'failed'}: {cmd}")
        print("-" * 50)

if __name__ == "__main__":
    main()
```

## Best Practices for LLM Planning

### Prompt Design Best Practices

1. **Be Specific and Clear**
   - Use precise language in prompts
   - Clearly define the expected output format
   - Specify constraints and requirements explicitly

2. **Provide Examples**
   - Include few-shot examples for complex tasks
   - Show both correct and incorrect examples
   - Use examples relevant to the robot's domain

3. **Include Context**
   - Provide environmental context
   - Specify robot capabilities and limitations
   - Include relevant state information

4. **Validate Outputs**
   - Always validate LLM outputs before execution
   - Implement safety checks and constraints
   - Use structured output formats (JSON) for parsing

### Safety and Reliability Best Practices

1. **Multi-Layer Validation**
   ```python
   def validate_plan_comprehensive(plan):
       # Layer 1: Format validation
       if not validate_format(plan):
           return False, "Invalid format"

       # Layer 2: Safety validation
       if not validate_safety(plan):
           return False, "Safety check failed"

       # Layer 3: Feasibility validation
       if not validate_feasibility(plan):
           return False, "Plan not feasible"

       return True, "Valid plan"
   ```

2. **Error Recovery**
   - Implement graceful degradation
   - Provide alternative plans when primary plan fails
   - Include human-in-the-loop for critical decisions

3. **Monitoring and Feedback**
   - Monitor plan execution in real-time
   - Collect feedback for plan improvement
   - Log all planning decisions for analysis

### Performance Optimization

1. **Caching Common Plans**
   ```python
   class PlanCache:
       def __init__(self):
           self.cache = {}

       def get_cached_plan(self, command):
           return self.cache.get(command)

       def cache_plan(self, command, plan):
           self.cache[command] = plan
   ```

2. **Plan Reuse and Adaptation**
   - Adapt existing plans for similar tasks
   - Use plan templates for common patterns
   - Implement plan libraries for frequent tasks

3. **Incremental Planning**
   - Plan only the next few steps initially
   - Replan as new information becomes available
   - Balance planning depth with responsiveness

## Summary

Cognitive planning with LLMs represents a significant advancement in robotic task execution, enabling natural language interaction with complex robotic systems. By properly implementing prompt engineering techniques, planning algorithms, and safety measures, we can create robust systems that translate human intent into reliable robotic action.

The key to success lies in combining the reasoning capabilities of LLMs with structured planning approaches, always maintaining safety and reliability as top priorities.