# Quickstart Guide: Vision-Language-Action Setup

**Created**: 2025-12-11
**Feature**: Module 4 — Vision-Language-Action (VLA)
**Status**: Draft

## Prerequisites

- Ubuntu 22.04 LTS (recommended for ROS 2 compatibility)
- Python 3.8+ with pip
- At least 8GB RAM and 20GB free disk space
- Microphone for voice input
- Graphics card with OpenGL 3.3+ support (for robot simulation)

## Setting up ROS 2 Environment

### Installing ROS 2 Humble Hawksbill

1. **Set up the repository**:
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   ```

2. **Install ROS 2 packages**:
   ```bash
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep2 python3-rosinstall python3-vcstool
   ```

3. **Initialize rosdep**:
   ```bash
   sudo rosdep init
   rosdep update
   ```

4. **Source the ROS 2 environment**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Setting up Whisper for Voice Recognition

### Installing OpenAI Whisper

1. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install ffmpeg
   pip3 install torch torchaudio
   ```

2. **Install Whisper**:
   ```bash
   pip3 install openai-whisper
   ```

3. **Download Whisper model** (for local processing):
   ```bash
   python3 -c "import whisper; whisper.load_model('base')"
   ```

### Alternative: Using OpenAI API

1. **Install OpenAI Python package**:
   ```bash
   pip3 install openai
   ```

2. **Set up your API key**:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

## Setting up LLM Integration

### Option 1: OpenAI GPT Integration

1. **Install required packages**:
   ```bash
   pip3 install openai python-dotenv
   ```

2. **Create .env file**:
   ```bash
   echo "OPENAI_API_KEY=your-api-key-here" > .env
   ```

### Option 2: Anthropic Claude Integration

1. **Install required packages**:
   ```bash
   pip3 install anthropic python-dotenv
   ```

2. **Update .env file**:
   ```bash
   echo "ANTHROPIC_API_KEY=your-api-key-here" >> .env
   ```

## Basic VLA System Test

### Testing Voice Recognition

1. **Create a simple voice recognition script**:
   ```python
   import whisper
   import os

   # Load model
   model = whisper.load_model("base")

   # Transcribe audio file
   result = model.transcribe("path/to/your/audio.wav")
   print(result["text"])
   ```

### Testing LLM Integration

1. **Create a simple LLM interaction script**:
   ```python
   from openai import OpenAI
   import os
   from dotenv import load_dotenv

   load_dotenv()
   client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

   response = client.chat.completions.create(
     model="gpt-4",
     messages=[
       {"role": "user", "content": "Convert this command to ROS 2 action sequence: 'Move forward 2 meters'"}
     ]
   )

   print(response.choices[0].message.content)
   ```

### Testing ROS 2 Action Integration

1. **Verify ROS 2 installation**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

2. **Test basic ROS 2 functionality**:
   ```bash
   # Terminal 1
   ros2 run demo_nodes_cpp talker

   # Terminal 2
   ros2 run demo_nodes_py listener
   ```

## VLA System Integration Example

### Complete Voice-to-Action Pipeline

```python
#!/usr/bin/env python3
import whisper
import rospy
from std_msgs.msg import String
from openai import OpenAI
import os
from dotenv import load_dotenv

class VLAPipeline:
    def __init__(self):
        # Initialize Whisper model
        self.whisper_model = whisper.load_model("base")

        # Initialize OpenAI client
        load_dotenv()
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

        # Initialize ROS 2
        rospy.init_node('vla_pipeline')
        self.command_pub = rospy.Publisher('robot_commands', String, queue_size=10)

    def voice_to_action(self, audio_path):
        # Step 1: Voice recognition
        transcription = self.whisper_model.transcribe(audio_path)
        command_text = transcription["text"]

        # Step 2: LLM processing for action planning
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "user", "content": f"Convert this command to ROS 2 action sequence: '{command_text}'"}
            ]
        )

        action_sequence = response.choices[0].message.content

        # Step 3: Publish action to robot
        self.command_pub.publish(String(data=action_sequence))

        return action_sequence

if __name__ == '__main__':
    vla = VLAPipeline()
    # Example usage
    # action = vla.voice_to_action("path/to/audio.wav")
    rospy.spin()
```

## Troubleshooting Common Issues

### Voice Recognition Problems
1. **No audio input detected**:
   - Check microphone permissions and connections
   - Verify audio input device with `arecord -l`
   - Test with: `arecord -d 3 test.wav && aplay test.wav`

2. **Poor recognition accuracy**:
   - Use higher quality Whisper model (medium, large)
   - Ensure quiet environment during recording
   - Preprocess audio for noise reduction

### LLM Integration Issues
1. **API connection errors**:
   - Verify API key is correctly set in environment
   - Check internet connectivity
   - Confirm API service is available

2. **Invalid action sequences**:
   - Implement validation layer for LLM outputs
   - Use structured prompts to constrain output format
   - Add safety checks before executing commands

### ROS 2 Connection Problems
1. **Cannot connect to ROS master**:
   - Ensure ROS environment is sourced
   - Check ROS_DOMAIN_ID if multiple systems present
   - Verify network configuration for multi-robot setups

2. **Action execution failures**:
   - Verify robot is properly connected and calibrated
   - Check robot capabilities match requested actions
   - Monitor robot status and error messages

## Next Steps

After completing the setup:

1. Review the VLA module chapters:
   - Chapter 1: LLMs & Robotics Convergence
   - Chapter 2: Voice-to-Action with Whisper
   - Chapter 3: Cognitive Planning with LLMs
   - Chapter 4: Capstone Project: Autonomous Humanoid

2. Practice with simple voice commands and observe the VLA pipeline in action

3. Experiment with different LLM prompts to improve action sequence generation

4. Test the complete system with simulated or physical humanoid robots