# VLA System Interface Contracts

## Overview
This document defines the interfaces and contracts for the Vision-Language-Action (VLA) system components. These contracts specify how different parts of the system interact with each other.

## Voice Recognition Interface

### Whisper Service Contract
- **Input**: Audio data (WAV, MP3, or raw audio stream)
- **Output**: Transcribed text with confidence score
- **Error Conditions**:
  - Invalid audio format
  - Audio too short/long
  - Network error (for API-based processing)
  - Recognition confidence below threshold

### Voice Command Processor
- **Input**: Raw audio, user context
- **Output**: Structured command object with intent and parameters
- **Processing Requirements**:
  - Noise reduction preprocessing
  - Command validation
  - Intent classification
  - Parameter extraction

## LLM Cognitive Planning Interface

### LLM Request Interface
- **Input**: Natural language command, context, robot capabilities
- **Output**: Structured action sequence with execution plan
- **Processing Requirements**:
  - Context preservation
  - Task decomposition
  - Action validation
  - Safety checks

### Prompt Template Contract
- **Input Format**: System message, user command, context
- **Output Format**: Valid ROS 2 action sequence in JSON format
- **Validation Requirements**:
  - All actions must be valid ROS 2 commands
  - Parameters must be within robot capabilities
  - Sequence must be executable in order

## ROS 2 Action Interface

### Action Sequence Executor
- **Input**: Action sequence (list of ROS 2 action calls)
- **Output**: Execution status, completion feedback
- **Error Handling**:
  - Individual action failure
  - Robot unavailable
  - Safety constraint violation

### Robot Capability Interface
- **Query**: Available actions and parameters
- **Response**: List of supported ROS 2 actions with parameter schemas
- **Update**: Capability changes when robot configuration changes

## VLA System Integration Contracts

### Voice-to-Action Pipeline
```
[Audio Input] -> [Voice Recognition] -> [NLU Processing] -> [LLM Planning] -> [Action Execution] -> [Robot Feedback]
```

### Error Propagation Contract
- Voice recognition errors -> User feedback with retry option
- LLM planning errors -> Alternative plan generation or user clarification request
- Action execution errors -> Robot status update and recovery attempt

### Context Management Contract
- Session context maintained across multiple interactions
- Context includes: conversation history, robot state, user preferences
- Context expires after 30 minutes of inactivity

## Data Format Contracts

### Voice Command Format
```json
{
  "id": "unique-identifier",
  "text": "transcribed text",
  "confidence": 0.85,
  "timestamp": "2025-12-11T10:30:00Z",
  "user_context": "conversation history"
}
```

### Action Sequence Format
```json
{
  "id": "sequence-identifier",
  "commands": [
    {
      "action_type": "move_base",
      "parameters": {
        "x": 2.0,
        "y": 1.5,
        "theta": 0.0
      }
    }
  ],
  "robot_id": "humanoid-robot-01",
  "priority": 3,
  "estimated_duration": 15.0
}
```

### Robot Status Format
```json
{
  "robot_id": "humanoid-robot-01",
  "status": "idle|executing|error",
  "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
  "battery_level": 0.85,
  "capabilities": ["move_base", "gripper_control", "head_control"]
}
```

## Performance Contracts

### Response Time Requirements
- Voice recognition: <2 seconds for 10-second audio clip
- LLM planning: <5 seconds for simple action sequence
- Action execution start: <1 second after sequence validation

### Accuracy Requirements
- Voice recognition: >90% accuracy in quiet environment
- Intent classification: >85% accuracy for common commands
- Action sequence success: >80% for valid commands

## Security and Safety Contracts

### Input Validation
- All voice commands must be validated against allowed action types
- LLM outputs must be sanitized before execution
- Robot safety constraints must be enforced at execution time

### Privacy Protection
- Voice data should be processed locally when possible
- User interaction data should be anonymized
- No personal information should be stored without consent