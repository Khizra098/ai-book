# Quickstart: Vision-Language-Action (VLA) Implementation

**Feature**: 1-vla | **Date**: 2025-12-16 | **Plan**: [plan.md](./plan.md)

## Overview

This quickstart guide provides a rapid introduction to implementing Vision-Language-Action (VLA) systems that convert voice commands into autonomous robot actions using AI-powered cognitive planning. The guide covers essential setup and first steps for creating voice-controlled humanoid robots.

## Prerequisites

Before starting with the VLA implementation, ensure you have:

### System Requirements
- Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- 8GB+ RAM (16GB+ recommended for development)
- 50GB+ available disk space
- Modern CPU with SSE4.1 support
- Internet connection for API access and package downloads

### Software Requirements
- Python 3.8 or higher
- ROS 2 Humble Hawksbill
- Node.js 18+ LTS (for Docusaurus documentation)
- OpenAI API key
- Git version control system
- Docker (optional, for containerized development)

### Hardware Requirements (for testing)
- Microphone for voice input
- Humanoid robot platform (or simulation environment like Gazebo)
- GPU recommended for local Whisper model (optional)

## Setup Process

### 1. Clone and Navigate to Repository
```bash
git clone [repository-url]
cd ai-native-book
```

### 2. Install ROS 2 Dependencies
```bash
# Follow ROS 2 Humble installation guide
# Ubuntu:
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-colcon-common-extensions

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 3. Set Up Python Environment
```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate  # On Windows: vla_env\Scripts\activate
pip install --upgrade pip

# Install required packages
pip install openai
pip install SpeechRecognition
pip install ros2
pip install numpy
pip install pyaudio  # May require additional system packages
```

### 4. Configure OpenAI API
```bash
# Create .env file in project root
echo "OPENAI_API_KEY=your_api_key_here" > .env

# Or set environment variable
export OPENAI_API_KEY="your_api_key_here"
```

## First Steps with Voice Recognition

### 1. Basic Voice Command Recognition
Start with a simple voice recognition example to understand the basics:

```python
# example: basic_voice_recognition.py
import openai
import speech_recognition as sr
import os
from dotenv import load_dotenv

load_dotenv()

# Configure OpenAI API
openai.api_key = os.getenv("OPENAI_API_KEY")

def recognize_speech():
    """
    Basic speech recognition using Whisper
    """
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    print("Please speak a command...")
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        # Use Whisper for speech recognition
        transcript = recognizer.recognize_whisper(audio, language="en")
        print(f"Recognized: {transcript}")
        return transcript
    except sr.UnknownValueError:
        print("Could not understand audio")
        return None
    except sr.RequestError as e:
        print(f"Error with speech recognition service: {e}")
        return None

if __name__ == "__main__":
    recognized_text = recognize_speech()
    if recognized_text:
        print(f"Processing command: {recognized_text}")
```

### 2. Running the Basic Example
```bash
# Make sure your virtual environment is activated
source vla_env/bin/activate
python basic_voice_recognition.py
```

## First Steps with Cognitive Planning

### 1. Basic LLM Integration for Command Understanding
Learn to use LLMs for interpreting natural language commands:

```python
# example: basic_command_planning.py
import openai
import json
import os
from dotenv import load_dotenv

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

def plan_actions_from_command(command):
    """
    Use LLM to convert natural language command to action plan
    """
    system_prompt = """
    You are a robot command interpreter. Convert natural language commands
    into structured robot actions. Respond with a JSON object containing:
    - action_type: navigation, manipulation, interaction, etc.
    - parameters: specific parameters for the action
    - priority: 1-5 priority level
    """

    user_prompt = f"Convert this command to robot actions: {command}"

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        temperature=0.1
    )

    return response.choices[0].message.content

def parse_action_plan(llm_response):
    """
    Parse the LLM response into structured action plan
    """
    try:
        # Extract JSON from LLM response
        start = llm_response.find('{')
        end = llm_response.rfind('}') + 1
        json_str = llm_response[start:end]
        return json.loads(json_str)
    except:
        return {"error": "Could not parse action plan"}

if __name__ == "__main__":
    command = "Move forward 2 meters"
    llm_response = plan_actions_from_command(command)
    action_plan = parse_action_plan(llm_response)
    print(f"Command: {command}")
    print(f"Action Plan: {json.dumps(action_plan, indent=2)}")
```

## First Steps with Robot Control

### 1. Basic ROS 2 Action Client
Create a simple ROS 2 action client to execute planned actions:

```python
# example: basic_robot_controller.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Initialize action clients for different robot capabilities
        # This is a simplified example - actual implementation would use
        # specific action types for navigation, manipulation, etc.
        self.get_logger().info('Robot controller initialized')

    def send_navigation_goal(self, x, y, theta):
        """
        Send navigation goal to robot
        """
        # Implementation would use ROS 2 navigation action interface
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    # Spin node to keep it alive
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete VLA Pipeline

### 1. Integrated Example
Combine all components into a complete voice-to-action pipeline:

```python
# example: complete_vla_pipeline.py
import openai
import speech_recognition as sr
import json
import os
from dotenv import load_dotenv

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

class VLAPipeline:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def recognize_voice_command(self):
        """
        Capture and recognize voice command
        """
        print("Listening for command...")
        with self.microphone as source:
            audio = self.recognizer.listen(source)

        try:
            transcript = self.recognizer.recognize_whisper_api(audio, api_key=openai.api_key)
            print(f"Recognized: {transcript}")
            return transcript
        except Exception as e:
            print(f"Error recognizing speech: {e}")
            return None

    def plan_actions(self, command):
        """
        Plan robot actions based on command
        """
        system_prompt = """
        You are a robot command interpreter. Convert natural language commands
        into structured robot actions. Respond with a JSON object containing:
        - action_type: navigation, manipulation, interaction, etc.
        - parameters: specific parameters for the action
        - priority: 1-5 priority level
        """

        user_prompt = f"Convert this command to robot actions: {command}"

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1
        )

        return response.choices[0].message.content

    def execute_pipeline(self):
        """
        Execute complete VLA pipeline
        """
        command = self.recognize_voice_command()
        if command:
            print(f"Processing command: {command}")
            action_plan = self.plan_actions(command)
            print(f"Generated action plan: {action_plan}")
            # In a real implementation, this would connect to ROS 2
            # to execute the planned actions on the robot

if __name__ == "__main__":
    pipeline = VLAPipeline()
    pipeline.execute_pipeline()
```

## Running the Examples

### 1. Execute the Complete Pipeline
```bash
# Make sure your environment is activated
source vla_env/bin/activate

# Run the complete pipeline
python complete_vla_pipeline.py
```

### 2. Test Individual Components
```bash
# Test voice recognition only
python basic_voice_recognition.py

# Test command planning only
python basic_command_planning.py
```

## Validation and Testing

### 1. Voice Recognition Validation
- Verify that spoken commands are accurately transcribed
- Test with different accents and speaking speeds
- Validate confidence scores for accuracy assessment

### 2. Command Planning Validation
- Test with various natural language commands
- Verify that generated action plans are valid and executable
- Check that the LLM properly interprets complex multi-step commands

### 3. System Integration Validation
- End-to-end testing of voice-to-action pipeline
- Verify proper error handling at each stage
- Test timeout and recovery mechanisms

## Troubleshooting Common Issues

### Voice Recognition Issues
- **No audio input**: Check microphone permissions and connections
- **Poor recognition**: Ensure quiet environment and clear speech
- **API errors**: Verify OpenAI API key and account limits

### LLM Integration Issues
- **Poor command interpretation**: Refine system prompts
- **Invalid action plans**: Implement better response parsing
- **Rate limiting**: Add appropriate delays and retry logic

### ROS 2 Integration Issues
- **Connection problems**: Verify ROS 2 network configuration
- **Action interface errors**: Check action message definitions
- **Permission issues**: Ensure proper ROS 2 setup and permissions

## Next Steps

After completing this quickstart:

1. Proceed to Chapter 1: Voice-to-Action with OpenAI Whisper for in-depth coverage
2. Move to Chapter 2: Cognitive Planning using LLMs for ROS 2 for planning systems
3. Complete Chapter 3: Capstone: Autonomous Humanoid executing tasks for full implementation
4. Experiment with the complete integrated examples
5. Try customizing the pipeline for your specific robot platform

## Resources

- OpenAI API Documentation: https://platform.openai.com/docs
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Whisper API Documentation: https://platform.openai.com/docs/api-reference/whisper
- Module-specific examples and code in the repository