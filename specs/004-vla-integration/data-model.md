# Data Model: Vision-Language-Action (VLA) Module

## Core Entities

### VoiceCommand
- **Description**: Natural language input from a human user that needs to be processed and converted to robotic actions
- **Attributes**:
  - id: Unique identifier for the command
  - text: The transcribed text from voice input
  - confidence: Confidence score from voice recognition (0.0-1.0)
  - timestamp: When the command was received
  - source: Audio source identifier
  - status: Processing status (pending, processed, failed)
- **Validation Rules**:
  - Text must not be empty
  - Confidence must be between 0.0 and 1.0
  - Timestamp must be in the past or present
- **Relationships**: One-to-many with ActionSequence (a command can generate multiple action sequences)

### LLMProcessing
- **Description**: The cognitive layer that interprets natural language and generates planning sequences
- **Attributes**:
  - id: Unique identifier for the processing instance
  - input_command: The original voice command text
  - prompt_template: Template used for LLM interaction
  - response: The LLM's response containing action plan
  - processing_time: Time taken for LLM processing
  - confidence: Confidence in the generated plan
- **Validation Rules**:
  - Input command must reference a valid VoiceCommand
  - Response must contain valid action sequences
  - Processing time must be positive
- **Relationships**: One-to-one with VoiceCommand, one-to-many with ActionSequence

### ROS2ActionSequence
- **Description**: The executable commands that control the humanoid robot's behavior
- **Attributes**:
  - id: Unique identifier for the action sequence
  - commands: List of individual ROS 2 action commands
  - robot_id: Identifier of the target robot
  - priority: Execution priority (1-5)
  - estimated_duration: Estimated time for completion
  - status: Execution status (pending, executing, completed, failed)
- **Validation Rules**:
  - Commands must be valid ROS 2 action calls
  - Robot ID must reference a valid robot
  - Priority must be between 1 and 5
- **Relationships**: Many-to-one with LLMProcessing, many-to-one with HumanoidRobot

### HumanoidRobot
- **Description**: The physical or simulated robot platform that executes the planned actions
- **Attributes**:
  - id: Unique identifier for the robot
  - name: Human-readable name for the robot
  - capabilities: List of supported actions/functions
  - status: Current operational status (idle, busy, error)
  - position: Current position in environment
  - battery_level: Current battery level (0.0-1.0)
- **Validation Rules**:
  - Name must be unique
  - Capabilities must be valid ROS 2 action types
  - Battery level must be between 0.0 and 1.0
- **Relationships**: One-to-many with ROS2ActionSequence

### VLAInteractionSession
- **Description**: A session representing a complete interaction from voice input to robot execution
- **Attributes**:
  - id: Unique identifier for the session
  - start_time: When the session started
  - end_time: When the session ended (null if active)
  - user_id: Identifier of the user initiating the session
  - status: Current session status (active, completed, failed)
  - context: Current conversation context/state
- **Validation Rules**:
  - Start time must be before end time (if end time exists)
  - Status must be one of the defined values
- **Relationships**: One-to-many with VoiceCommand, one-to-many with ROS2ActionSequence

## State Transitions

### VoiceCommand States
- **pending** → **processing**: When command is sent to Whisper for recognition
- **processing** → **recognized**: When Whisper successfully transcribes audio
- **processing** → **failed**: When Whisper fails to recognize audio
- **recognized** → **planning**: When LLM planning begins
- **recognized** → **rejected**: When command is invalid or unclear

### ROS2ActionSequence States
- **pending** → **executing**: When sequence is sent to robot for execution
- **executing** → **completed**: When robot successfully completes all actions
- **executing** → **failed**: When robot fails to complete actions
- **executing** → **interrupted**: When execution is manually stopped

### VLAInteractionSession States
- **active** → **completed**: When all commands in session are processed
- **active** → **failed**: When session encounters unrecoverable error
- **active** → **timeout**: When session exceeds maximum duration

## Data Flow Patterns

### Voice-to-Action Flow
1. Voice input captured → VoiceCommand created (pending state)
2. Whisper processes audio → VoiceCommand moves to recognized state
3. LLM processes recognized text → LLMProcessing created
4. LLM generates action sequence → ROS2ActionSequence created
5. Action sequence sent to robot → Execution begins

### Context Management
- VLAInteractionSession maintains conversation context
- Context includes previous commands, robot state, and user preferences
- Context is passed to LLM for coherent multi-turn interactions

## Validation Rules

### Cross-Entity Validation
- A VoiceCommand must have a corresponding LLMProcessing instance before creating ROS2ActionSequence
- A ROS2ActionSequence must reference a valid HumanoidRobot that is in 'idle' status
- An action sequence can only be executed if the robot has sufficient battery level (>0.2)

### Business Rules
- Maximum session duration: 30 minutes
- Maximum number of pending action sequences per robot: 5
- Minimum confidence threshold for voice recognition: 0.7
- Minimum confidence threshold for LLM planning: 0.6