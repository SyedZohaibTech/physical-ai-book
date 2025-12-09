---
title: "2. Voice Commands with OpenAI Whisper"
sidebar_label: "Whisper Voice Commands"
sidebar_position: 2
---

# 2. Voice Commands with OpenAI Whisper

The first step in any Vision-Language-Action (VLA) pipeline is often to convert human input into a format the robot can understand. For spoken commands, this means **speech-to-text**. In this chapter, we'll integrate OpenAI's powerful **Whisper** model into our ROS 2 system to enable our robot to understand voice commands.

## Why Whisper?

Whisper is a general-purpose speech recognition model developed by OpenAI. It is trained on a large dataset of diverse audio and is capable of transcribing speech into text with high accuracy, even in noisy environments or with different accents. Crucially for robotics, it can run locally on reasonable hardware, providing low-latency transcription.

## The Speech-to-Text ROS 2 Node

We will create a simple ROS 2 Python node that:
1.  Captures audio from the microphone.
2.  Processes the audio with the Whisper model to generate text.
3.  Publishes the transcribed text to a ROS 2 topic.

### 1. Audio Capture with `sounddevice`

The `sounddevice` Python library provides a simple way to capture audio from your system's microphone.

```python
# Minimal example of audio capture
import sounddevice as sd
import numpy as np

duration = 3  # seconds
fs = 16000    # Sample rate (Hz) - Whisper prefers 16kHz

print("Recording for 3 seconds...")
recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32')
sd.wait() # Wait until recording is finished
print("Recording complete.")

# You now have 'recording' as a numpy array of audio samples
```

### 2. Transcribing with Whisper

Once you have the audio as a NumPy array, you can pass it to the Whisper model.

```python
import whisper

# Load the desired Whisper model (e.g., 'base', 'small', 'medium', 'large')
# 'base' is good for local inference on many CPUs
model = whisper.load_model("base") 

# Make sure the audio is in the correct format (16kHz, mono)
# 'recording' should be a numpy array of float32 samples
audio_segment = recording.flatten() 

print("Transcribing...")
result = model.transcribe(audio_segment)
transcribed_text = result["text"]
print(f"Transcribed: {transcribed_text}")
```

### 3. Integrating into ROS 2

Now, let's combine these into a ROS 2 node. This node will run continuously, listening for a trigger (e.g., a specific key press or a command on another topic), record a short audio clip, transcribe it, and then publish the result.

```python
# ~/ros2_ws/src/voice_command_pkg/voice_command_pkg/whisper_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import whisper
import threading
import time

class WhisperNode(Node):

    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)
        self.declare_parameter('audio_duration', 3.0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('whisper_model', 'base')

        self.audio_duration = self.get_parameter('audio_duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.whisper_model_name = self.get_parameter('whisper_model').value

        self.get_logger().info(f"Loading Whisper model: {self.whisper_model_name}...")
        self.whisper_model = whisper.load_model(self.whisper_model_name)
        self.get_logger().info("Whisper model loaded.")

        # Example: Trigger recording every 5 seconds for demonstration
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Whisper Node has been started. Recording triggered every 5s.')

    def timer_callback(self):
        self.get_logger().info("Recording audio...")
        try:
            recording = sd.rec(int(self.audio_duration * self.sample_rate),
                               samplerate=self.sample_rate,
                               channels=1,
                               dtype='float32')
            sd.wait() # Wait until recording is finished
            self.get_logger().info("Recording complete. Transcribing...")

            audio_segment = recording.flatten()
            result = self.whisper_model.transcribe(audio_segment)
            transcribed_text = result["text"]

            msg = String()
            msg.data = transcribed_text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')

        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setup and Running

1.  **Install dependencies**:
    ```bash
    pip install sounddevice numpy openai-whisper
    # You might need portaudio development files:
    # sudo apt-get install libportaudio2 python3-pyaudio
    ```
2.  **Create ROS 2 package**:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python voice_command_pkg --dependencies rclpy std_msgs
    ```
3.  **Place code**: Put the `whisper_node.py` file into `~/ros2_ws/src/voice_command_pkg/voice_command_pkg/`.
4.  **Update `setup.py`**: Add an entry point for your node in `~/ros2_ws/src/voice_command_pkg/setup.py` under `entry_points`:
    ```python
    entry_points={
        'console_scripts': [
            'whisper_node = voice_command_pkg.whisper_node:main',
        ],
    },
    ```
5.  **Build**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
6.  **Run**:
    ```bash
    ros2 run voice_command_pkg whisper_node
    ```
7.  **Verify**: In another terminal, echo the topic:
    ```bash
    ros2 topic echo /voice_command
    ```
    Speak into your microphone when the node indicates it's recording. You should see your spoken words appear on the `/voice_command` topic.

This `whisper_node` now forms the crucial first link in our VLA chain, translating the messy world of human speech into clean, structured text that our robot's AI can process. In the next chapter, we'll see how to take this text and turn it into executable robot actions.
