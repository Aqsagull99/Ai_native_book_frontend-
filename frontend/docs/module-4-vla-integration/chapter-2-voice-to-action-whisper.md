# Chapter 2: Voice-to-Action with Whisper

## Introduction

Voice-to-Action systems represent a critical component of the Vision-Language-Action (VLA) framework, enabling robots to understand and respond to spoken natural language commands. OpenAI's Whisper model provides robust speech recognition capabilities that can convert human voice commands into text, which can then be processed by language models to generate appropriate robot actions.

This chapter explores the implementation of Whisper-based voice recognition systems for humanoid robots, covering model selection, configuration, speech processing techniques, and command interpretation methods. We'll examine how to build reliable voice-to-action pipelines that enable natural human-robot interaction.

## Whisper Model Selection and Configuration

### Overview of Whisper Models

OpenAI's Whisper model family offers different trade-offs between accuracy, speed, and computational requirements. Understanding these options is crucial for selecting the appropriate model for your robotic application:

1. **tiny**: Fastest model, suitable for real-time applications with limited computational resources
   - Size: ~75MB
   - Performance: Good for clear audio in controlled environments
   - Use case: Simple command recognition in quiet environments

2. **base**: Balanced model offering improved accuracy over tiny
   - Size: ~145MB
   - Performance: Better accuracy for varied accents and background noise
   - Use case: General-purpose voice commands in typical environments

3. **small**: Higher accuracy model suitable for more challenging recognition tasks
   - Size: ~485MB
   - Performance: Significantly better accuracy, especially for complex audio
   - Use case: Complex command recognition, multiple speakers

4. **medium**: High accuracy model for professional applications
   - Size: ~1.5GB
   - Performance: Excellent accuracy across diverse conditions
   - Use case: Mission-critical applications requiring high reliability

5. **large**: Highest accuracy model, including large-v1 and large-v2 variants
   - Size: ~3.0GB
   - Performance: State-of-the-art accuracy, handles challenging audio conditions
   - Use case: Applications where accuracy is paramount

### Model Selection Criteria for Robotics

When selecting a Whisper model for robotic applications, consider these factors:

- **Latency Requirements**: Real-time applications may require smaller, faster models
- **Accuracy Needs**: Complex command interpretation may require higher accuracy models
- **Computational Resources**: Available GPU/CPU resources and memory constraints
- **Environmental Conditions**: Background noise, multiple speakers, acoustic properties
- **Language Requirements**: Multi-language support needs

### Whisper Configuration for Robotics

Optimal Whisper configuration for robotic applications involves several key parameters:

```python
import whisper

# Load model with appropriate configuration
model = whisper.load_model("base")  # Choose model size based on requirements

# Configuration options for robotic applications
options = {
    "language": "en",  # Specify language for better accuracy
    "task": "transcribe",  # Use 'transcribe' for voice commands
    "temperature": 0.0,  # Lower temperature for more consistent results
    "best_of": 5,  # Number of candidate generations to consider
    "beam_size": 5,  # Beam size for beam search decoding
    "patience": 1.0,  # Patience factor for beam search
    "length_penalty": 1.0,  # Length penalty for decoding
    "suppress_blank": True,  # Suppress blank outputs at the beginning
    "suppress_tokens": [-1],  # Suppress specific tokens
    "without_timestamps": True,  # Don't need timestamps for command recognition
    "max_initial_timestamp": 1.0,  # Maximum initial timestamp
    "word_timestamps": False,  # Don't need word-level timestamps for commands
}

# Transcribe audio with configuration
result = model.transcribe("audio_file.wav", **options)
transcription = result["text"]
confidence = result["avg_logprob"]  # Confidence score
```

## Speech-to-Text Processing Techniques

### Audio Preprocessing

Effective voice command recognition requires proper audio preprocessing to optimize Whisper's performance:

```python
import librosa
import numpy as np
import soundfile as sf

def preprocess_audio(audio_path, target_sr=16000):
    """
    Preprocess audio for optimal Whisper performance
    """
    # Load audio
    audio, sr = librosa.load(audio_path, sr=None)

    # Resample to Whisper's expected sample rate (16kHz)
    if sr != target_sr:
        audio = librosa.resample(audio, orig_sr=sr, target_sr=target_sr)

    # Normalize audio to prevent clipping
    audio = librosa.util.normalize(audio)

    # Apply noise reduction if needed
    # (Simple spectral gating approach)
    if np.mean(audio**2) < 0.001:  # If very quiet, might be noise
        # Apply noise reduction techniques here
        pass

    # Save processed audio
    processed_path = audio_path.replace('.wav', '_processed.wav')
    sf.write(processed_path, audio, target_sr)

    return processed_path

# Example usage
processed_audio = preprocess_audio("robot_command.wav")
```

### Real-Time Audio Processing

For real-time voice command recognition, implement streaming audio processing:

```python
import pyaudio
import wave
import threading
import queue
import time

class RealTimeWhisperProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper expects 16kHz
        self.record_seconds = 3  # Process audio in 3-second chunks

    def start_recording(self):
        """Start recording audio in a separate thread"""
        self.is_recording = True
        recording_thread = threading.Thread(target=self._record_audio)
        recording_thread.start()

    def _record_audio(self):
        """Internal method to record audio"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        print("Recording... Speak now!")

        while self.is_recording:
            frames = []
            for _ in range(0, int(self.rate / self.chunk_size * self.record_seconds)):
                data = stream.read(self.chunk_size)
                frames.append(data)

            # Convert frames to numpy array
            audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize

            # Add to queue for processing
            self.audio_queue.put(audio_data)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def process_audio(self):
        """Process audio chunks and recognize speech"""
        while self.is_recording or not self.audio_queue.empty():
            try:
                audio_chunk = self.audio_queue.get(timeout=1)

                # Transcribe the audio chunk
                result = self.model.transcribe(audio_chunk, language="en")
                transcription = result["text"].strip()

                if transcription:  # If we have a transcription
                    print(f"Recognized: {transcription}")
                    self.handle_command(transcription)

            except queue.Empty:
                continue

    def handle_command(self, command):
        """Process recognized command"""
        if command.lower().startswith("move") or command.lower().startswith("go"):
            print(f"Processing movement command: {command}")
            # Implement command processing logic here
        elif command.lower().startswith("pick") or command.lower().startswith("grasp"):
            print(f"Processing manipulation command: {command}")
            # Implement command processing logic here
        else:
            print(f"Processing general command: {command}")
            # Implement general command processing logic here

# Example usage
processor = RealTimeWhisperProcessor(model_size="base")
processor.start_recording()

# Process audio in main thread
try:
    processor.process_audio()
except KeyboardInterrupt:
    print("Stopping...")
    processor.is_recording = False
```

### Confidence-Based Recognition

Implement confidence thresholding to ensure reliable command recognition:

```python
def transcribe_with_confidence(model, audio_path, confidence_threshold=0.7):
    """
    Transcribe audio with confidence thresholding
    """
    result = model.transcribe(audio_path, language="en")

    # Calculate confidence (Whisper provides log probability)
    avg_logprob = result.get("avg_logprob", -10.0)
    no_speech_prob = result.get("no_speech_prob", 1.0)

    # Convert log probability to a more intuitive confidence score
    confidence = max(0, min(1, (avg_logprob + 10) / 10))

    # Consider as valid transcription if confidence is above threshold
    # and no_speech_prob is below threshold
    if confidence > confidence_threshold and no_speech_prob < 0.5:
        return result["text"].strip(), confidence
    else:
        return "", confidence  # Return empty string for low confidence

# Example usage
model = whisper.load_model("base")
command, confidence = transcribe_with_confidence(model, "command.wav")

if command:
    print(f"Command: {command} (confidence: {confidence:.2f})")
else:
    print(f"Could not recognize command (confidence: {confidence:.2f})")
```

## Command Interpretation and Parsing

### Natural Language Command Processing

Once Whisper converts speech to text, the next step is to interpret the command and extract actionable elements:

```python
import re
from typing import Dict, List, Tuple, Optional

class VoiceCommandInterpreter:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'move': [
                r'move\s+(?P<direction>forward|backward|left|right|up|down)\s+(?P<distance>[\d.]+)\s*(?P<unit>meters?|cm|steps?)',
                r'go\s+(?P<direction>forward|backward|left|right|up|down)\s+(?P<distance>[\d.]+)\s*(?P<unit>meters?|cm|steps?)',
                r'walk\s+(?P<direction>forward|backward|left|right)\s+(?P<distance>[\d.]+)\s*(?P<unit>meters?|cm|steps?)',
            ],
            'move_to': [
                r'go\s+to\s+(?P<location>\w+)',
                r'move\s+to\s+(?P<location>\w+)',
                r'go\s+to\s+the\s+(?P<location>\w+)',
            ],
            'grasp': [
                r'pick\s+up\s+(?P<object>\w+)',
                r'grasp\s+(?P<object>\w+)',
                r'grab\s+(?P<object>\w+)',
                r'pick\s+(?P<object>\w+)',
            ],
            'place': [
                r'place\s+(?P<object>\w+)\s+on\s+(?P<location>\w+)',
                r'put\s+(?P<object>\w+)\s+on\s+(?P<location>\w+)',
                r'put\s+(?P<object>\w+)\s+in\s+(?P<location>\w+)',
            ],
            'speak': [
                r'say\s+(?P<text>.+)',
                r'speak\s+(?P<text>.+)',
                r'tell\s+me\s+(?P<text>.+)',
            ],
            'open': [
                r'open\s+(?P<object>\w+)',
                r'open\s+the\s+(?P<object>\w+)',
            ],
            'close': [
                r'close\s+(?P<object>\w+)',
                r'close\s+the\s+(?P<object>\w+)',
            ]
        }

    def parse_command(self, text: str) -> Optional[Dict]:
        """
        Parse a natural language command and extract structured information
        """
        text = text.lower().strip()

        for action_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    # Extract parameters
                    params = match.groupdict()
                    return {
                        'action': action_type,
                        'parameters': params,
                        'raw_text': text
                    }

        # If no pattern matches, return as generic command
        return {
            'action': 'generic',
            'parameters': {'text': text},
            'raw_text': text
        }

    def validate_command(self, parsed_command: Dict) -> bool:
        """
        Validate if the parsed command is appropriate for the robot
        """
        action = parsed_command['action']
        params = parsed_command['parameters']

        # Example validation rules
        if action == 'move':
            if 'distance' in params:
                try:
                    distance = float(params['distance'])
                    if distance > 10:  # Max 10 meters
                        return False
                except ValueError:
                    return False

        return True

# Example usage
interpreter = VoiceCommandInterpreter()

# Test command parsing
test_commands = [
    "Move forward 2 meters",
    "Go to the kitchen",
    "Pick up the red cup",
    "Place the cup on the table",
    "Say hello to everyone"
]

for cmd in test_commands:
    parsed = interpreter.parse_command(cmd)
    print(f"Command: '{cmd}' -> {parsed}")
```

### Integration with LLM for Advanced Interpretation

For more complex command interpretation, integrate with LLMs to handle ambiguous or complex requests:

```python
from openai import OpenAI
import os
from dotenv import load_dotenv

class AdvancedVoiceCommandInterpreter:
    def __init__(self):
        load_dotenv()
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.basic_interpreter = VoiceCommandInterpreter()

    def interpret_complex_command(self, command_text: str) -> Dict:
        """
        Use LLM to interpret complex or ambiguous commands
        """
        # First try basic interpretation
        basic_result = self.basic_interpreter.parse_command(command_text)

        # If basic interpretation is generic or ambiguous, use LLM
        if basic_result['action'] == 'generic' or not basic_result:
            # Use LLM to interpret the command
            prompt = f"""
            You are a command interpreter for a humanoid robot. Convert the following human command
            into a structured robot command with clear action and parameters.

            Human Command: "{command_text}"

            Respond in JSON format with:
            - action: the robot action (move, move_to, grasp, place, speak, open, close, etc.)
            - parameters: relevant parameters for the action
            - explanation: brief explanation of your interpretation

            Example response:
            {{
                "action": "move",
                "parameters": {{"direction": "forward", "distance": "2", "unit": "meters"}},
                "explanation": "Interpreted 'move forward' as a movement command"
            }}
            """

            try:
                response = self.client.chat.completions.create(
                    model="gpt-4",
                    messages=[{"role": "user", "content": prompt}],
                    response_format={"type": "json_object"}
                )

                import json
                result = json.loads(response.choices[0].message.content)
                result['raw_text'] = command_text
                return result
            except Exception as e:
                print(f"LLM interpretation failed: {e}")
                # Fallback to basic interpretation
                return basic_result
        else:
            return basic_result

# Example usage
advanced_interpreter = AdvancedVoiceCommandInterpreter()
complex_command = "Could you please go to the living room and bring me the book from the table?"
result = advanced_interpreter.interpret_complex_command(complex_command)
print(f"Complex command interpretation: {result}")
```

## Practical Examples with Whisper Implementation for Humanoid Robots

### Complete Voice-to-Action Pipeline

Here's a complete implementation of a voice-to-action pipeline for humanoid robots:

```python
import whisper
import pyaudio
import wave
import numpy as np
import threading
import queue
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from your_robot_interfaces.srv import ExecuteAction

class VoiceToActionPipeline(Node):
    def __init__(self):
        super().__init__('voice_to_action_pipeline')

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("base")

        # Initialize command interpreter
        self.interpreter = AdvancedVoiceCommandInterpreter()

        # Audio recording parameters
        self.chunk_size = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.record_seconds = 3

        # Audio processing
        self.audio_queue = queue.Queue()
        self.is_listening = False
        self.pyaudio_instance = pyaudio.PyAudio()

        # ROS 2 publishers and services
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)
        self.action_client = self.create_client(ExecuteAction, 'execute_action')

        # Timer for processing audio
        self.timer = self.create_timer(0.1, self.process_audio_queue)

    def start_listening(self):
        """Start listening for voice commands"""
        self.is_listening = True
        self.get_logger().info("Starting to listen for voice commands...")

        # Start audio recording in separate thread
        recording_thread = threading.Thread(target=self._record_audio)
        recording_thread.start()

    def _record_audio(self):
        """Internal method to record audio"""
        stream = self.pyaudio_instance.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        self.get_logger().info("Audio recording started. Speak now!")

        while self.is_listening:
            frames = []
            for _ in range(0, int(self.rate / self.chunk_size * self.record_seconds)):
                data = stream.read(self.chunk_size)
                frames.append(data)

            # Convert frames to numpy array
            audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
            audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize

            # Add to queue for processing
            self.audio_queue.put(audio_data)

        stream.stop_stream()
        stream.close()

    def process_audio_queue(self):
        """Process audio from queue and convert to robot commands"""
        while not self.audio_queue.empty():
            try:
                audio_chunk = self.audio_queue.get_nowait()

                # Transcribe audio chunk
                result = self.whisper_model.transcribe(audio_chunk, language="en")
                transcription = result["text"].strip()

                if transcription and len(transcription) > 3:  # Valid transcription
                    self.get_logger().info(f"Recognized: {transcription}")

                    # Interpret the command
                    interpreted_cmd = self.interpreter.interpret_complex_command(transcription)

                    # Validate command
                    if self.basic_interpreter.validate_command(interpreted_cmd):
                        # Publish command to robot
                        cmd_msg = String()
                        cmd_msg.data = f"{interpreted_cmd['action']} {interpreted_cmd['parameters']}"
                        self.command_publisher.publish(cmd_msg)

                        self.get_logger().info(f"Published command: {cmd_msg.data}")
                    else:
                        self.get_logger().warn(f"Invalid command: {transcription}")

            except queue.Empty:
                break  # No more audio to process

    def stop_listening(self):
        """Stop listening for voice commands"""
        self.is_listening = False
        self.pyaudio_instance.terminate()
        self.get_logger().info("Stopped listening for voice commands")

def main(args=None):
    rclpy.init(args=args)
    voice_pipeline = VoiceToActionPipeline()

    try:
        voice_pipeline.start_listening()
        rclpy.spin(voice_pipeline)
    except KeyboardInterrupt:
        voice_pipeline.get_logger().info("Interrupted by user")
    finally:
        voice_pipeline.stop_listening()
        voice_pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Voice Command Processing with Error Handling

Implement robust error handling for voice command processing:

```python
import logging
from enum import Enum

class VoiceCommandStatus(Enum):
    SUCCESS = "success"
    RECOGNITION_FAILED = "recognition_failed"
    INTERPRETATION_FAILED = "interpretation_failed"
    VALIDATION_FAILED = "validation_failed"
    EXECUTION_FAILED = "execution_failed"

class RobustVoiceProcessor:
    def __init__(self):
        self.whisper_model = whisper.load_model("base")
        self.interpreter = AdvancedVoiceCommandInterpreter()
        self.logger = logging.getLogger(__name__)

    def process_voice_command(self, audio_path: str) -> Tuple[VoiceCommandStatus, str, Dict]:
        """
        Process voice command with comprehensive error handling
        Returns: (status, message, command_data)
        """
        try:
            # Step 1: Transcribe audio
            result = self.whisper_model.transcribe(audio_path, language="en")
            transcription = result["text"].strip()

            if not transcription or len(transcription) < 3:
                return VoiceCommandStatus.RECOGNITION_FAILED, "Could not recognize speech", {}

            # Calculate confidence
            avg_logprob = result.get("avg_logprob", -10.0)
            confidence = max(0, min(1, (avg_logprob + 10) / 10))

            if confidence < 0.5:  # Low confidence
                return VoiceCommandStatus.RECOGNITION_FAILED, f"Low confidence recognition: {transcription}", {}

            self.logger.info(f"Recognized: {transcription} (confidence: {confidence:.2f})")

            # Step 2: Interpret command
            try:
                interpreted_cmd = self.interpreter.interpret_complex_command(transcription)
            except Exception as e:
                self.logger.error(f"Command interpretation failed: {e}")
                return VoiceCommandStatus.INTERPRETATION_FAILED, f"Could not interpret command: {transcription}", {}

            # Step 3: Validate command
            if not self.interpreter.basic_interpreter.validate_command(interpreted_cmd):
                return VoiceCommandStatus.VALIDATION_FAILED, f"Invalid command: {interpreted_cmd}", interpreted_cmd

            # Step 4: Return successful result
            return VoiceCommandStatus.SUCCESS, f"Successfully processed: {transcription}", interpreted_cmd

        except Exception as e:
            self.logger.error(f"Voice command processing failed: {e}")
            return VoiceCommandStatus.EXECUTION_FAILED, f"Processing error: {str(e)}", {}

# Example usage with error handling
processor = RobustVoiceProcessor()
status, message, command_data = processor.process_voice_command("command.wav")

if status == VoiceCommandStatus.SUCCESS:
    print(f"Command processed successfully: {command_data}")
else:
    print(f"Command processing failed: {message}")

    # Handle different error types appropriately
    if status == VoiceCommandStatus.RECOGNITION_FAILED:
        print("Suggestion: Please speak more clearly or try again")
    elif status == VoiceCommandStatus.INTERPRETATION_FAILED:
        print("Suggestion: Use clearer command format")
    elif status == VoiceCommandStatus.VALIDATION_FAILED:
        print("Suggestion: Command is not supported or outside robot capabilities")
```

## Summary

Voice-to-Action systems using OpenAI Whisper enable natural and intuitive human-robot interaction. By properly selecting and configuring Whisper models, implementing effective speech processing techniques, and developing robust command interpretation systems, we can create humanoid robots that respond to spoken commands with high accuracy and reliability.

The key to successful implementation lies in balancing accuracy requirements with computational constraints, implementing proper error handling and validation, and designing systems that can handle the natural ambiguity and variability of human speech.