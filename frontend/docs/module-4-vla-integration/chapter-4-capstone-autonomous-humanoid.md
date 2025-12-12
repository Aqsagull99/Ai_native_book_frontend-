# Chapter 4: Capstone Project: Autonomous Humanoid

## Introduction

The capstone project brings together all the concepts learned in previous chapters to create a complete Vision-Language-Action (VLA) system for an autonomous humanoid robot. This project integrates voice recognition, natural language understanding, cognitive planning, and robotic action execution into a unified system that can respond to voice commands by executing complex multi-step tasks autonomously.

This chapter serves as a comprehensive guide to implementing the complete VLA system, demonstrating how all components work together to create an intelligent, responsive humanoid robot capable of understanding and executing natural language commands in real-world environments.

## Capstone Project Requirements

### Functional Requirements

The autonomous humanoid system must fulfill the following functional requirements:

1. **Voice Command Reception**: The system shall accept and process voice commands from users in natural language.

2. **Speech Recognition**: The system shall accurately convert spoken commands to text using OpenAI Whisper or similar technology, with a minimum accuracy of 90% in quiet environments.

3. **Natural Language Understanding**: The system shall interpret the meaning and intent of voice commands, identifying objects, locations, and actions requested.

4. **Cognitive Planning**: The system shall generate executable action sequences from high-level commands using LLM-based planning.

5. **Action Execution**: The system shall execute planned actions through ROS 2 interfaces, controlling the humanoid robot's movements and manipulations.

6. **Environmental Awareness**: The system shall maintain awareness of its environment, including object locations and navigable paths.

7. **Safety Validation**: The system shall validate all planned actions for safety before execution.

8. **Feedback Provision**: The system shall provide feedback to users about command status and execution progress.

### Performance Requirements

1. **Response Time**: The system shall process and begin executing simple commands within 5 seconds of receiving the voice input.

2. **Execution Accuracy**: The system shall successfully complete 80% of valid commands without human intervention.

3. **Recognition Accuracy**: Voice recognition shall achieve 90% accuracy in controlled environments.

4. **System Availability**: The system shall maintain 95% uptime during operational periods.

### Integration Requirements

1. **Whisper Integration**: The system shall integrate with OpenAI Whisper for voice recognition.

2. **LLM Integration**: The system shall interface with large language models for cognitive planning.

3. **ROS 2 Integration**: The system shall use ROS 2 for robot control and communication.

4. **Hardware Compatibility**: The system shall be compatible with humanoid robot platforms supporting ROS 2.

## Voice Command Processing Integration

### Voice Input Pipeline

The voice command processing pipeline integrates all components from Chapter 2 to create a robust voice-to-action system:

```python
import whisper
import threading
import queue
import numpy as np
from typing import Dict, Optional

class VoiceCommandProcessor:
    def __init__(self, whisper_model_size="base", confidence_threshold=0.7):
        self.whisper_model = whisper.load_model(whisper_model_size)
        self.confidence_threshold = confidence_threshold
        self.audio_queue = queue.Queue()
        self.command_queue = queue.Queue()
        self.is_active = False

        # Initialize components from previous chapters
        self.command_interpreter = AdvancedVoiceCommandInterpreter()
        self.cognitive_planner = LLMPlanningWorkflow()

    def start_listening(self):
        """Start the voice command processing pipeline"""
        self.is_active = True
        audio_thread = threading.Thread(target=self._audio_capture_loop)
        audio_thread.daemon = True
        audio_thread.start()

    def _audio_capture_loop(self):
        """Continuous audio capture and processing"""
        import pyaudio

        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        while self.is_active:
            # Capture audio chunk
            frames = []
            for _ in range(0, int(16000 / 1024 * 2)):  # 2-second chunks
                data = stream.read(1024)
                frames.append(data)

            # Process audio chunk
            audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / 32768.0

            # Transcribe with Whisper
            result = self.whisper_model.transcribe(audio_data, language="en")
            transcription = result["text"].strip()

            # Check confidence
            avg_logprob = result.get("avg_logprob", -10.0)
            confidence = max(0, min(1, (avg_logprob + 10) / 10))

            if transcription and confidence > self.confidence_threshold:
                # Process the command
                self._process_command(transcription)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _process_command(self, command_text: str):
        """Process a recognized voice command through the complete pipeline"""
        try:
            # Step 1: Interpret the command
            interpreted_cmd = self.command_interpreter.interpret_complex_command(command_text)

            if not interpreted_cmd:
                print(f"Could not interpret command: {command_text}")
                return

            # Step 2: Validate the command
            if not self.command_interpreter.basic_interpreter.validate_command(interpreted_cmd):
                print(f"Invalid command: {interpreted_cmd}")
                return

            # Step 3: Plan the action sequence using cognitive planning
            success = self.cognitive_planner.execute_planning_workflow(command_text)

            if success:
                print(f"Command executed successfully: {command_text}")
            else:
                print(f"Command execution failed: {command_text}")

        except Exception as e:
            print(f"Error processing command '{command_text}': {e}")

    def stop_listening(self):
        """Stop the voice command processing"""
        self.is_active = False
```

### Command Validation and Filtering

Implement validation to ensure only appropriate commands are processed:

```python
class CommandValidator:
    def __init__(self):
        self.safe_actions = {
            'move', 'move_to', 'pick_up', 'place', 'speak',
            'open', 'close', 'navigate', 'wait', 'stop'
        }
        self.forbidden_keywords = ['dangerous', 'harm', 'injure', 'break']
        self.max_command_length = 200  # characters

    def validate_command(self, command_data: Dict) -> tuple[bool, str]:
        """
        Validate a command for safety and appropriateness
        Returns: (is_valid, reason)
        """
        # Check if action is safe
        action = command_data.get('action', '')
        if action not in self.safe_actions:
            return False, f"Action '{action}' is not in safe actions list"

        # Check for forbidden keywords
        raw_text = command_data.get('raw_text', '').lower()
        for keyword in self.forbidden_keywords:
            if keyword in raw_text:
                return False, f"Command contains forbidden keyword: {keyword}"

        # Check command length
        if len(raw_text) > self.max_command_length:
            return False, f"Command too long: {len(raw_text)} > {self.max_command_length}"

        # Validate parameters
        params = command_data.get('parameters', {})
        if not self._validate_parameters(action, params):
            return False, "Command parameters are invalid"

        return True, "Command is valid"

    def _validate_parameters(self, action: str, params: Dict) -> bool:
        """Validate action parameters"""
        if action == 'move':
            if 'distance' in params:
                try:
                    distance = float(params['distance'])
                    if distance > 10.0:  # Max 10 meters
                        return False
                except ValueError:
                    return False

        elif action == 'pick_up':
            if 'object' not in params:
                return False

        return True
```

### Error Handling and Recovery

Implement robust error handling for the voice command pipeline:

```python
class VoiceCommandErrorRecovery:
    def __init__(self):
        self.retry_attempts = 3
        self.timeout_seconds = 30

    def execute_with_recovery(self, command_text: str) -> bool:
        """
        Execute a command with error recovery mechanisms
        """
        for attempt in range(self.retry_attempts):
            try:
                # Execute the command
                success = self._execute_command(command_text)

                if success:
                    return True
                else:
                    print(f"Command failed on attempt {attempt + 1}")

            except Exception as e:
                print(f"Command error on attempt {attempt + 1}: {e}")

            if attempt < self.retry_attempts - 1:
                print("Retrying...")
                time.sleep(2)  # Wait before retry

        return False

    def _execute_command(self, command_text: str) -> bool:
        """Execute a single command with timeout"""
        import signal

        def timeout_handler(signum, frame):
            raise TimeoutError("Command execution timed out")

        # Set timeout
        signal.signal(signal.SIGALRM, timeout_handler)
        signal.alarm(self.timeout_seconds)

        try:
            # Execute the actual command
            # This would call the integrated pipeline
            return True  # Simplified for example
        finally:
            signal.alarm(0)  # Cancel the alarm
```

## LLM Planning Integration

### Unified Planning Interface

Create a unified interface that integrates all planning components:

```python
from openai import OpenAI
import json
from typing import List, Dict, Any

class UnifiedPlanningInterface:
    def __init__(self):
        self.client = OpenAI()  # Initialize OpenAI client
        self.environment_context = {
            "robot_capabilities": ["move", "grasp", "speak", "navigate", "manipulate"],
            "available_objects": [],
            "environment_map": {},
            "current_state": {}
        }

    def generate_plan(self, goal: str, context: Dict = None) -> Dict:
        """
        Generate a complete action plan using LLM-based cognitive planning
        """
        if context is None:
            context = self.environment_context

        prompt = f"""
        You are an AI planning agent for an autonomous humanoid robot. Generate a complete
        action plan to achieve the specified goal.

        Goal: {goal}
        Current Environment Context: {json.dumps(context, indent=2)}

        Planning Requirements:
        1. Break down the goal into executable actions
        2. Consider environmental constraints
        3. Validate against robot capabilities
        4. Include safety checks
        5. Plan for potential failures and recovery

        Return the plan in the following JSON format:
        {{
            "plan_id": "unique_plan_identifier",
            "goal": "{goal}",
            "actions": [
                {{
                    "id": "action_1",
                    "type": "action_type",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description of the action",
                    "preconditions": ["condition1", "condition2"],
                    "postconditions": ["condition1", "condition2"],
                    "success_criteria": "How to verify success",
                    "timeout": 30.0,
                    "retries": 3
                }}
            ],
            "dependencies": [
                {{"from": "action_1", "to": "action_2", "type": "execution_order"}}
            ],
            "metadata": {{
                "estimated_duration": 120.0,
                "complexity_score": 0.7,
                "risk_level": "low",
                "confidence": 0.9
            }}
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"},
                temperature=0.3  # Lower temperature for more consistent outputs
            )

            plan = json.loads(response.choices[0].message.content)
            return plan

        except Exception as e:
            print(f"Plan generation error: {e}")
            # Return a fallback plan
            return self._create_fallback_plan(goal)

    def _create_fallback_plan(self, goal: str) -> Dict:
        """Create a simple fallback plan when LLM fails"""
        return {
            "plan_id": "fallback_plan",
            "goal": goal,
            "actions": [],
            "dependencies": [],
            "metadata": {
                "estimated_duration": 0.0,
                "complexity_score": 0.0,
                "risk_level": "unknown",
                "confidence": 0.0
            }
        }

    def update_context(self, new_context: Dict):
        """Update the environment context"""
        self.environment_context.update(new_context)
```

### Plan Validation and Optimization

Validate and optimize plans before execution:

```python
class PlanValidator:
    def __init__(self):
        self.robot_capabilities = {
            "max_speed": 1.0,  # m/s
            "max_payload": 5.0,  # kg
            "max_reach": 1.5,  # meters
            "navigation_accuracy": 0.1  # meters
        }

    def validate_plan(self, plan: Dict) -> tuple[bool, List[str]]:
        """
        Validate a plan for feasibility and safety
        Returns: (is_valid, list_of_issues)
        """
        issues = []

        # Check if plan has required structure
        if "actions" not in plan:
            issues.append("Plan missing 'actions' field")
            return False, issues

        actions = plan["actions"]

        # Validate each action
        for i, action in enumerate(actions):
            action_issues = self._validate_action(action, i)
            issues.extend(action_issues)

        # Check for plan-level issues
        if len(actions) == 0:
            issues.append("Plan contains no actions")

        # Calculate estimated duration
        total_duration = sum(action.get("timeout", 10.0) for action in actions)
        if total_duration > 3600:  # 1 hour limit
            issues.append(f"Plan duration too long: {total_duration}s")

        return len(issues) == 0, issues

    def _validate_action(self, action: Dict, index: int) -> List[str]:
        """Validate a single action"""
        issues = []

        if "type" not in action:
            issues.append(f"Action {index}: Missing action type")
            return issues

        action_type = action["type"]

        # Validate based on action type
        if action_type == "move":
            params = action.get("parameters", {})
            if "x" not in params or "y" not in params:
                issues.append(f"Action {index}: Move action missing coordinates")
        elif action_type == "pick_up":
            params = action.get("parameters", {})
            if "object" not in params:
                issues.append(f"Action {index}: Pick up action missing object parameter")

        # Check timeout values
        timeout = action.get("timeout", 30.0)
        if timeout <= 0 or timeout > 300:  # 5 minute max
            issues.append(f"Action {index}: Invalid timeout value: {timeout}")

        return issues

    def optimize_plan(self, plan: Dict) -> Dict:
        """
        Optimize the plan for better execution
        """
        optimized_plan = plan.copy()
        actions = optimized_plan["actions"]

        # Remove redundant actions
        optimized_actions = self._remove_redundant_actions(actions)

        # Optimize action order where possible
        optimized_actions = self._optimize_action_order(optimized_actions)

        optimized_plan["actions"] = optimized_actions
        return optimized_plan

    def _remove_redundant_actions(self, actions: List[Dict]) -> List[Dict]:
        """Remove redundant or unnecessary actions"""
        # Implementation to remove redundant actions
        return actions  # Simplified for example

    def _optimize_action_order(self, actions: List[Dict]) -> List[Dict]:
        """Optimize the order of actions for efficiency"""
        # Implementation to optimize action order
        return actions  # Simplified for example
```

## ROS 2 Action Sequence Execution

### ROS 2 Integration Layer

Create the interface between the planning system and ROS 2:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from your_robot_interfaces.action import MoveTo, ManipulateObject, NavigateTo
import time

class ROS2ExecutionInterface(Node):
    def __init__(self):
        super().__init__('vla_execution_interface')

        # Action clients for different robot capabilities
        self.move_client = ActionClient(self, MoveTo, 'move_to_action')
        self.manipulation_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        self.navigation_client = ActionClient(self, NavigateTo, 'navigate_to')

        # Publishers for other commands
        self.speech_publisher = self.create_publisher(String, 'robot_speech', 10)

        # Wait for action servers to be available
        self.get_logger().info("Waiting for action servers...")
        self.move_client.wait_for_server()
        self.manipulation_client.wait_for_server()
        self.navigation_client.wait_for_server()
        self.get_logger().info("All action servers available")

    def execute_action_sequence(self, action_sequence: List[Dict]) -> bool:
        """
        Execute a sequence of actions through ROS 2
        """
        success = True

        for i, action in enumerate(action_sequence):
            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action['type']}")

            action_success = self._execute_single_action(action)

            if not action_success:
                self.get_logger().error(f"Action failed: {action}")
                success = False
                break

        return success

    def _execute_single_action(self, action: Dict) -> bool:
        """
        Execute a single action via ROS 2
        """
        action_type = action['type']

        try:
            if action_type == 'MoveTo':
                return self._execute_move_to(action['parameters'])
            elif action_type == 'NavigateTo':
                return self._execute_navigate_to(action['parameters'])
            elif action_type == 'PickUp':
                return self._execute_pick_up(action['parameters'])
            elif action_type == 'Place':
                return self._execute_place(action['parameters'])
            elif action_type == 'Speak':
                return self._execute_speak(action['parameters'])
            else:
                self.get_logger().error(f"Unknown action type: {action_type}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action {action_type}: {e}")
            return False

    def _execute_move_to(self, params: Dict) -> bool:
        """Execute MoveTo action"""
        goal_msg = MoveTo.Goal()
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position.x = float(params.get('x', 0.0))
        goal_msg.target_pose.position.y = float(params.get('y', 0.0))
        goal_msg.target_pose.position.z = float(params.get('z', 0.0))

        # Send goal and wait for result
        future = self.move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return result.success if result else False

    def _execute_navigate_to(self, params: Dict) -> bool:
        """Execute NavigateTo action"""
        goal_msg = NavigateTo.Goal()
        goal_msg.target_location = params.get('location', '')

        future = self.navigation_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return result.success if result else False

    def _execute_pick_up(self, params: Dict) -> bool:
        """Execute PickUp action"""
        goal_msg = ManipulateObject.Goal()
        goal_msg.object_name = params.get('object', '')
        goal_msg.action = 'pick_up'

        future = self.manipulation_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return result.success if result else False

    def _execute_place(self, params: Dict) -> bool:
        """Execute Place action"""
        goal_msg = ManipulateObject.Goal()
        goal_msg.object_name = params.get('object', '')
        goal_msg.target_location = params.get('location', '')
        goal_msg.action = 'place'

        future = self.manipulation_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        return result.success if result else False

    def _execute_speak(self, params: Dict) -> bool:
        """Execute Speak action"""
        text = params.get('text', '')
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
        return True  # Publishing is considered successful
```

### Execution Monitoring and Feedback

Monitor execution and provide feedback:

```python
class ExecutionMonitor:
    def __init__(self, ros_node: ROS2ExecutionInterface):
        self.ros_node = ros_node
        self.execution_history = []
        self.current_execution_id = None

    def execute_with_monitoring(self, plan: Dict) -> Dict:
        """
        Execute a plan with monitoring and feedback
        """
        execution_id = f"exec_{int(time.time())}"
        self.current_execution_id = execution_id

        start_time = time.time()
        self.ros_node.get_logger().info(f"Starting execution: {execution_id}")

        # Initialize execution record
        execution_record = {
            "execution_id": execution_id,
            "plan_id": plan.get("plan_id", "unknown"),
            "start_time": start_time,
            "actions": [],
            "success": False,
            "error": None
        }

        try:
            # Execute each action in the plan
            actions = plan.get("actions", [])
            for i, action in enumerate(actions):
                action_result = self._execute_monitored_action(action, i, len(actions))
                execution_record["actions"].append(action_result)

                # Check if action failed and whether to continue
                if not action_result["success"]:
                    if action.get("retries", 0) > 0:
                        # Attempt retry
                        action_result = self._retry_action(action, action_result)
                        execution_record["actions"][-1] = action_result

                    if not action_result["success"]:
                        execution_record["error"] = action_result.get("error", "Action failed")
                        break

            execution_record["success"] = all(
                action["success"] for action in execution_record["actions"]
            )

        except Exception as e:
            execution_record["error"] = str(e)
            execution_record["success"] = False

        finally:
            execution_record["end_time"] = time.time()
            execution_record["duration"] = execution_record["end_time"] - start_time
            self.execution_history.append(execution_record)

            self.ros_node.get_logger().info(
                f"Execution {execution_id} completed. Success: {execution_record['success']}, "
                f"Duration: {execution_record['duration']:.2f}s"
            )

        return execution_record

    def _execute_monitored_action(self, action: Dict, index: int, total: int) -> Dict:
        """Execute a single action with monitoring"""
        start_time = time.time()

        self.ros_node.get_logger().info(
            f"Executing action {index+1}/{total}: {action['type']} - {action['description']}"
        )

        try:
            success = self.ros_node._execute_single_action(action)

            result = {
                "action_id": action.get("id", f"action_{index}"),
                "type": action["type"],
                "success": success,
                "start_time": start_time,
                "end_time": time.time(),
                "duration": time.time() - start_time,
                "parameters": action.get("parameters", {}),
                "description": action.get("description", "")
            }

            if success:
                self.ros_node.get_logger().info(f"Action {index+1} completed successfully")
            else:
                self.ros_node.get_logger().error(f"Action {index+1} failed")

        except Exception as e:
            result = {
                "action_id": action.get("id", f"action_{index}"),
                "type": action["type"],
                "success": False,
                "start_time": start_time,
                "end_time": time.time(),
                "duration": time.time() - start_time,
                "error": str(e),
                "parameters": action.get("parameters", {}),
                "description": action.get("description", "")
            }
            self.ros_node.get_logger().error(f"Action {index+1} error: {e}")

        return result

    def _retry_action(self, action: Dict, failed_result: Dict) -> Dict:
        """Retry a failed action"""
        max_retries = action.get("retries", 0)
        retry_count = 0

        while retry_count < max_retries:
            retry_count += 1
            self.ros_node.get_logger().info(f"Retrying action (attempt {retry_count}/{max_retries})")

            time.sleep(1)  # Brief delay before retry
            retry_result = self._execute_monitored_action(action, 0, 1)

            if retry_result["success"]:
                self.ros_node.get_logger().info(f"Action succeeded on retry {retry_count}")
                return retry_result

        return failed_result
```

## Complete System Integration Examples

### Main System Integration

Here's how all components come together in the complete system:

```python
class CompleteVLASystem:
    def __init__(self):
        # Initialize all components
        self.voice_processor = VoiceCommandProcessor()
        self.planning_interface = UnifiedPlanningInterface()
        self.plan_validator = PlanValidator()
        self.ros_interface = ROS2ExecutionInterface()
        self.execution_monitor = ExecutionMonitor(self.ros_interface)

        # Initialize ROS 2
        if not rclpy.ok():
            rclpy.init()

    def process_voice_command(self, command_text: str) -> Dict:
        """
        Complete pipeline: Voice command -> Plan -> Execute -> Monitor
        """
        result = {
            "command": command_text,
            "success": False,
            "stages": {
                "interpretation": False,
                "planning": False,
                "validation": False,
                "execution": False
            },
            "execution_record": None
        }

        try:
            # Stage 1: Interpret the command
            interpreted_cmd = self.voice_processor.command_interpreter.interpret_complex_command(command_text)
            result["stages"]["interpretation"] = True

            if not interpreted_cmd:
                raise Exception("Could not interpret command")

            # Stage 2: Generate plan using LLM
            plan = self.planning_interface.generate_plan(command_text)
            result["stages"]["planning"] = True

            # Stage 3: Validate the plan
            is_valid, issues = self.plan_validator.validate_plan(plan)
            if not is_valid:
                raise Exception(f"Plan validation failed: {', '.join(issues)}")
            result["stages"]["validation"] = True

            # Stage 4: Optimize the plan
            optimized_plan = self.plan_validator.optimize_plan(plan)

            # Stage 5: Execute the plan with monitoring
            execution_record = self.execution_monitor.execute_with_monitoring(optimized_plan)
            result["stages"]["execution"] = execution_record["success"]
            result["execution_record"] = execution_record
            result["success"] = execution_record["success"]

        except Exception as e:
            result["error"] = str(e)
            self.ros_interface.get_logger().error(f"Pipeline error: {e}")

        return result

    def start_autonomous_mode(self):
        """
        Start the complete autonomous system
        """
        self.ros_interface.get_logger().info("Starting autonomous humanoid system")

        # Start voice command processing
        self.voice_processor.start_listening()

        try:
            # Keep the system running
            rclpy.spin(self.ros_interface)
        except KeyboardInterrupt:
            self.ros_interface.get_logger().info("Shutting down autonomous system")
        finally:
            self.voice_processor.stop_listening()
            self.ros_interface.destroy_node()
            rclpy.shutdown()

# Example usage of the complete system
def main():
    # Create the complete VLA system
    vla_system = CompleteVLASystem()

    # Example commands to demonstrate the system
    test_commands = [
        "Go to the kitchen and bring me a glass of water",
        "Move the red book from the table to the shelf",
        "Navigate to the living room and turn on the lamp"
    ]

    print("Testing complete VLA system with sample commands...")

    for command in test_commands:
        print(f"\nProcessing command: '{command}'")
        result = vla_system.process_voice_command(command)

        print(f"Command result: {result['success']}")
        print(f"Stages completed: {result['stages']}")

        if result['execution_record']:
            exec_record = result['execution_record']
            print(f"Execution duration: {exec_record['duration']:.2f}s")
            print(f"Actions attempted: {len(exec_record['actions'])}")
            print(f"Actions successful: {sum(1 for a in exec_record['actions'] if a['success'])}")

if __name__ == "__main__":
    main()
```

### System Configuration and Setup

```python
class SystemConfiguration:
    def __init__(self):
        self.config = {
            # Whisper settings
            "whisper": {
                "model_size": "base",
                "confidence_threshold": 0.7,
                "sample_rate": 16000
            },

            # LLM settings
            "llm": {
                "model": "gpt-4",
                "temperature": 0.3,
                "max_tokens": 1000
            },

            # Planning settings
            "planning": {
                "max_plan_duration": 3600,  # 1 hour
                "max_actions": 50,
                "validation_enabled": True
            },

            # Execution settings
            "execution": {
                "max_retries": 3,
                "action_timeout": 30.0,
                "monitoring_enabled": True
            },

            # Safety settings
            "safety": {
                "enable_validation": True,
                "forbidden_actions": ["jump", "run_fast"],
                "max_speed": 1.0
            }
        }

    def load_from_file(self, config_path: str):
        """Load configuration from file"""
        import json
        with open(config_path, 'r') as f:
            file_config = json.load(f)
        self.config.update(file_config)

    def save_to_file(self, config_path: str):
        """Save configuration to file"""
        import json
        with open(config_path, 'w') as f:
            json.dump(self.config, f, indent=2)

    def get(self, key_path: str, default=None):
        """
        Get configuration value using dot notation
        Example: get('whisper.model_size')
        """
        keys = key_path.split('.')
        value = self.config

        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default

        return value

    def set(self, key_path: str, value):
        """
        Set configuration value using dot notation
        """
        keys = key_path.split('.')
        config_ref = self.config

        for key in keys[:-1]:
            if key not in config_ref:
                config_ref[key] = {}
            config_ref = config_ref[key]

        config_ref[keys[-1]] = value
```

## VLA System Architecture Diagram

As mentioned in Task T045, we need to create a diagram showing the complete VLA system architecture. Since I cannot create actual image files, I'll create a text description of what the diagram would show:

```
Complete VLA System Architecture:

┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Human User    │    │  Voice Command   │    │  Whisper Model  │
│                 │───▶│  Processing      │───▶│  (Speech Rec.)  │
│  (Speaks)       │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                             │
                             ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Environment    │    │  Command         │    │  LLM Cognitive  │
│  (Objects,      │    │  Interpretation  │    │  Planner        │
│   Locations)    │◀───│                  │───▶│  (GPT-4)        │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                      ┌──────────────────┐    ┌─────────────────┐
                      │  Plan Validation │───▶│  ROS 2 Action   │
                      │  & Optimization  │    │  Execution      │
                      └──────────────────┘    └─────────────────┘
                              │
                              ▼
                      ┌──────────────────┐
                      │  Robot Feedback  │
                      │  & Monitoring    │
                      └──────────────────┘

Data Flow:
- Voice → Text → Understanding → Planning → Action → Execution → Feedback
- Environment context feeds into planning and interpretation
- Execution feedback updates environment context
```

This architecture represents the complete flow from voice input to robotic action execution, incorporating all the components developed in the previous chapters.

## Summary

The capstone project demonstrates the complete integration of Vision-Language-Action capabilities in an autonomous humanoid robot. By combining voice recognition, natural language understanding, cognitive planning, and robotic execution, we create a system that can understand and respond to natural language commands with complex, multi-step behaviors.

The key to success in this capstone project is the seamless integration of all components, with proper error handling, safety validation, and feedback mechanisms to ensure reliable operation in real-world environments.