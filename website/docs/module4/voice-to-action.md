---
sidebar_position: 2
title: Voice-to-Action Systems for Humanoid Robots
---

# Voice-to-Action Systems for Humanoid Robots

Voice-to-Action systems enable humanoid robots to understand spoken language and convert it into executable actions. This capability is crucial for natural human-robot interaction, allowing users to command robots using everyday language rather than specialized interfaces.

## Understanding Voice-to-Action Systems

Voice-to-Action systems process natural language commands and translate them into specific robot behaviors. This involves multiple stages:

1. **Speech Recognition**: Converting audio to text
2. **Natural Language Understanding**: Interpreting the meaning
3. **Action Planning**: Generating executable actions
4. **Execution**: Performing the requested tasks

For humanoid robots, this creates a natural interaction modality that matches human communication patterns.

## Speech Recognition for Robotics

### 1. Audio Processing Pipeline

The basic audio processing pipeline for robotics:

```python
import numpy as np
import speech_recognition as sr
from queue import Queue
import threading
import time

class AudioProcessor:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.audio_queue = Queue()
        self.is_listening = False
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
    
    def start_listening(self):
        """Start continuous listening for commands"""
        self.is_listening = True
        self.listening_thread = threading.Thread(target=self._listen_continuously)
        self.listening_thread.start()
    
    def _listen_continuously(self):
        """Continuously listen for audio and put on queue"""
        with self.microphone as source:
            while self.is_listening:
                try:
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                    self.audio_queue.put(audio)
                except sr.WaitTimeoutError:
                    continue  # Keep listening
    
    def stop_listening(self):
        """Stop the listening process"""
        self.is_listening = False
        if hasattr(self, 'listening_thread'):
            self.listening_thread.join()
    
    def get_audio_text(self, audio):
        """Convert audio to text"""
        try:
            text = self.recognizer.recognize_google(audio)
            return text
        except sr.UnknownValueError:
            return None
        except sr.RequestError as e:
            print(f"Error with speech recognition service: {e}")
            return None
```

### 2. Robust Speech Recognition

For robotic applications, robust speech recognition is essential:

```python
import webrtcvad
import collections
import pyaudio
import numpy as np
from scipy import signal

class RobustSpeechRecognizer:
    def __init__(self):
        # Initialize VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Aggressive VAD
        
        # Audio parameters
        self.rate = 16000
        self.chunk_duration_ms = 30  # Supports 10, 20 and 30 (ms)
        self.chunk_size = int(self.rate * self.chunk_duration_ms / 1000)
        self.window_duration_ms = 2000
        self.p = pyaudio.PyAudio()
        
        # Ring buffer for audio chunks
        self.ring_buffer = collections.deque(maxlen=self.window_duration_ms // self.chunk_duration_ms)
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
    
    def detect_voice_activity(self, data, sample_rate):
        """Detect if audio chunk contains voice activity"""
        return self.vad.is_speech(data, sample_rate)
    
    def preprocess_audio(self, audio_data):
        """Preprocess audio to improve recognition quality"""
        # Apply noise reduction
        # Convert to appropriate format for speech recognition
        return audio_data
    
    def recognize_speech(self, audio_data):
        """Recognize speech from audio data"""
        try:
            # Convert to audio file format for recognition
            # This is a simplified version - in practice, you'd use proper audio conversion
            text = self.recognizer.recognize_google(audio_data)
            return text
        except Exception as e:
            print(f"Speech recognition error: {e}")
            return None
```

## Natural Language Understanding

### 1. Command Parsing

Parsing natural language commands into structured actions:

```python
import re
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class Command:
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: Optional[dict] = None

class CommandParser:
    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move': [
                r'go to (.+)',
                r'move to (.+)',
                r'walk to (.+)',
                r'go (.+)',
                r'navigate to (.+)'
            ],
            'grasp': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'pick (.+) up',
                r'grasp (.+)'
            ],
            'place': [
                r'put (.+) on (.+)',
                r'place (.+) on (.+)',
                r'put (.+) at (.+)',
                r'place (.+) at (.+)'
            ],
            'follow': [
                r'follow (.+)',
                r'follow me',
                r'come with me'
            ],
            'wait': [
                r'wait here',
                r'wait for me',
                r'stop',
                r'pause'
            ],
            'greet': [
                r'say hello to (.+)',
                r'greet (.+)',
                r'hello (.+)',
                r'wave to (.+)'
            ]
        }
    
    def parse_command(self, text: str) -> Optional[Command]:
        """Parse natural language text into a structured command"""
        text = text.lower().strip()
        
        for action, patterns in self.action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    groups = match.groups()
                    
                    if action == 'move':
                        return Command(action=action, location=groups[0])
                    elif action == 'grasp':
                        return Command(action=action, target=groups[0])
                    elif action == 'place':
                        return Command(action=action, target=groups[0], location=groups[1])
                    elif action == 'follow':
                        return Command(action=action, target=groups[0] if groups else 'user')
                    elif action == 'wait':
                        return Command(action=action)
                    elif action == 'greet':
                        return Command(action=action, target=groups[0])
        
        return None  # Command not recognized
```

### 2. Contextual Understanding

Understanding commands in context:

```python
class ContextualUnderstanding:
    def __init__(self):
        self.current_context = {
            'location': 'unknown',
            'objects': [],
            'people': [],
            'task': None
        }
        self.command_history = []
        
    def update_context(self, new_info):
        """Update the current context with new information"""
        self.current_context.update(new_info)
    
    def resolve_pronouns(self, text, context):
        """Resolve pronouns based on context"""
        # Simple pronoun resolution
        text = text.replace('it', context.get('last_object', 'it'))
        text = text.replace('there', context.get('last_location', 'there'))
        text = text.replace('him', context.get('last_person', 'him'))
        text = text.replace('her', context.get('last_person', 'her'))
        return text
    
    def disambiguate_command(self, command, context):
        """Resolve ambiguities in the command based on context"""
        if command.action == 'move' and command.location == 'there':
            command.location = context.get('last_location', command.location)
        
        if command.action == 'grasp' and command.target == 'it':
            command.target = context.get('last_object', command.target)
        
        return command
    
    def parse_with_context(self, text, context):
        """Parse command considering the current context"""
        # Resolve pronouns
        resolved_text = self.resolve_pronouns(text, context)
        
        # Parse the resolved text
        parser = CommandParser()
        command = parser.parse_command(resolved_text)
        
        if command:
            # Disambiguate based on context
            command = self.disambiguate_command(command, context)
            
            # Update command history
            self.command_history.append(command)
        
        return command
```

## Action Planning from Voice Commands

### 1. Semantic Action Mapping

Mapping natural language commands to robot actions:

```python
import json
from typing import Dict, Any

class SemanticActionMapper:
    def __init__(self):
        # Load semantic mappings
        self.action_mappings = self.load_mappings()
        
        # Define action execution interfaces
        self.action_executors = {
            'move': self.execute_move,
            'grasp': self.execute_grasp,
            'place': self.execute_place,
            'follow': self.execute_follow,
            'wait': self.execute_wait,
            'greet': self.execute_greet
        }
    
    def load_mappings(self):
        """Load semantic mappings from configuration"""
        # This would typically load from a file or database
        return {
            'move': {
                'locations': {
                    'kitchen': '/map/kitchen',
                    'living room': '/map/living_room',
                    'bedroom': '/map/bedroom',
                    'dining room': '/map/dining_room',
                    'bathroom': '/map/bathroom',
                    'office': '/map/office'
                }
            },
            'grasp': {
                'objects': {
                    'cup': 'cup_grasp_pose',
                    'book': 'book_grasp_pose',
                    'bottle': 'bottle_grasp_pose',
                    'box': 'box_grasp_pose'
                }
            }
        }
    
    def map_command_to_action(self, command):
        """Map a parsed command to an executable action"""
        if command.action not in self.action_executors:
            raise ValueError(f"Unknown action: {command.action}")
        
        # Look up semantic information
        semantic_info = {}
        
        if command.action == 'move' and command.location:
            semantic_info['target_pose'] = self.action_mappings['move']['locations'].get(
                command.location, command.location
            )
        
        elif command.action == 'grasp' and command.target:
            semantic_info['object_name'] = command.target
            semantic_info['grasp_pose'] = self.action_mappings['grasp']['objects'].get(
                command.target, 'default_grasp'
            )
        
        elif command.action == 'place' and command.location:
            semantic_info['placement_surface'] = command.location
        
        # Create action specification
        action_spec = {
            'action_type': command.action,
            'semantic_info': semantic_info,
            'parameters': command.parameters or {}
        }
        
        return action_spec
    
    def execute_move(self, semantic_info):
        """Execute move action"""
        target_pose = semantic_info.get('target_pose')
        if target_pose:
            # Call navigation system
            print(f"Moving to: {target_pose}")
            # navigation_client.send_goal(target_pose)
        else:
            print("No target pose specified for move action")
    
    def execute_grasp(self, semantic_info):
        """Execute grasp action"""
        object_name = semantic_info.get('object_name')
        grasp_pose = semantic_info.get('grasp_pose', 'default_grasp')
        
        if object_name:
            print(f"Grasping {object_name} with {grasp_pose}")
            # manipulation_client.grasp_object(object_name, grasp_pose)
        else:
            print("No object specified for grasp action")
    
    def execute_place(self, semantic_info):
        """Execute place action"""
        placement_surface = semantic_info.get('placement_surface')
        if placement_surface:
            print(f"Placing object on {placement_surface}")
            # manipulation_client.place_object(placement_surface)
        else:
            print("No placement surface specified for place action")
    
    def execute_follow(self, semantic_info):
        """Execute follow action"""
        target = semantic_info.get('target', 'user')
        print(f"Following {target}")
        # following_client.start_following(target)
    
    def execute_wait(self, semantic_info):
        """Execute wait action"""
        print("Waiting...")
        # Stop robot motion
        # motion_client.stop()
    
    def execute_greet(self, semantic_info):
        """Execute greet action"""
        target = semantic_info.get('target', 'person')
        print(f"Greeting {target}")
        # gesture_client.wave()
        # speech_client.say(f"Hello, {target}!")
```

### 2. Task Planning System

Creating complex task plans from voice commands:

```python
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional

class TaskStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class TaskStep:
    action: str
    parameters: dict
    status: TaskStatus = TaskStatus.PENDING

class TaskPlanner:
    def __init__(self):
        self.current_tasks = []
        self.task_history = []
    
    def create_task_from_command(self, command):
        """Create a task plan from a voice command"""
        task_steps = []
        
        if command.action == 'fetch_and_place':
            # Complex task: fetch object and place it somewhere
            task_steps.append(TaskStep(
                action='navigate',
                parameters={'target_location': command.parameters.get('fetch_from')}
            ))
            task_steps.append(TaskStep(
                action='grasp',
                parameters={'object': command.target}
            ))
            task_steps.append(TaskStep(
                action='navigate',
                parameters={'target_location': command.parameters.get('place_at')}
            ))
            task_steps.append(TaskStep(
                action='place',
                parameters={'surface': command.parameters.get('place_at')}
            ))
        elif command.action == 'move':
            task_steps.append(TaskStep(
                action='navigate',
                parameters={'target_location': command.location}
            ))
        elif command.action == 'grasp':
            task_steps.append(TaskStep(
                action='navigate_to_object',
                parameters={'object': command.target}
            ))
            task_steps.append(TaskStep(
                action='grasp',
                parameters={'object': command.target}
            ))
        else:
            # Simple one-step tasks
            task_steps.append(TaskStep(
                action=command.action,
                parameters={
                    'target': command.target,
                    'location': command.location,
                    **(command.parameters or {})
                }
            ))
        
        return task_steps
    
    def execute_task(self, task_steps: List[TaskStep]):
        """Execute a sequence of task steps"""
        for i, step in enumerate(task_steps):
            print(f"Executing step {i+1}/{len(task_steps)}: {step.action}")
            
            # Execute the step
            success = self.execute_single_step(step)
            
            if not success:
                step.status = TaskStatus.FAILED
                print(f"Task failed at step {i+1}")
                return False
            
            step.status = TaskStatus.COMPLETED
            print(f"Step {i+1} completed")
        
        return True
    
    def execute_single_step(self, step: TaskStep):
        """Execute a single task step"""
        # This would interface with the robot's action execution system
        # For demonstration, we'll just print the action
        print(f"Executing: {step.action} with params: {step.parameters}")
        
        # Simulate execution (in reality, this would call robot services/actions)
        import time
        time.sleep(0.5)  # Simulate action execution time
        
        # Return success/failure (in reality, this would check robot feedback)
        return True
```

## Integration with Humanoid Control Systems

### 1. High-Level Voice Command Interface

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

class VoiceCommandInterface(Node):
    def __init__(self):
        super().__init__('voice_command_interface')
        
        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, '/voice_command_status', 10)
        self.speech_pub = self.create_publisher(String, '/tts_input', 10)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        
        # Voice processing components
        self.audio_processor = AudioProcessor()
        self.command_parser = CommandParser()
        self.action_mapper = SemanticActionMapper()
        self.task_planner = TaskPlanner()
        
        # State management
        self.is_listening = False
        self.current_task = None
        
        # Start voice processing
        self.voice_timer = self.create_timer(0.1, self.process_voice_commands)
    
    def start_listening(self):
        """Start listening for voice commands"""
        self.is_listening = True
        self.audio_processor.start_listening()
        self.publish_status("Listening for commands...")
    
    def stop_listening(self):
        """Stop listening for voice commands"""
        self.is_listening = False
        self.audio_processor.stop_listening()
        self.publish_status("Stopped listening")
    
    def process_voice_commands(self):
        """Process any queued voice commands"""
        if not self.is_listening:
            return
        
        # Check for new audio
        if not self.audio_processor.audio_queue.empty():
            audio = self.audio_processor.audio_queue.get()
            
            # Convert to text
            text = self.audio_processor.get_audio_text(audio)
            if text:
                self.get_logger().info(f"Heard: {text}")
                
                # Parse the command
                command = self.command_parser.parse_command(text)
                if command:
                    self.get_logger().info(f"Parsed command: {command}")
                    
                    # Map to action
                    action_spec = self.action_mapper.map_command_to_action(command)
                    
                    # Execute or plan the task
                    self.execute_voice_command(command, action_spec)
                else:
                    self.publish_status(f"Command not understood: {text}")
                    self.speak_response(f"Sorry, I didn't understand: {text}")
    
    def execute_voice_command(self, command, action_spec):
        """Execute a voice command"""
        if command.action in ['move', 'grasp', 'place', 'follow', 'wait', 'greet']:
            # Execute directly
            executor = self.action_mapper.action_executors[command.action]
            executor(action_spec.get('semantic_info', {}))
        else:
            # Create and execute task plan
            task_steps = self.task_planner.create_task_from_command(command)
            success = self.task_planner.execute_task(task_steps)
            
            if success:
                self.publish_status(f"Completed task: {command.action}")
                self.speak_response("Task completed successfully")
            else:
                self.publish_status(f"Failed to complete task: {command.action}")
                self.speak_response("Sorry, I couldn't complete that task")
    
    def publish_status(self, status):
        """Publish status message"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def speak_response(self, text):
        """Speak a response using TTS"""
        tts_msg = String()
        tts_msg.data = text
        self.speech_pub.publish(tts_msg)
```

## Advanced Voice Processing Techniques

### 1. Wake Word Detection

For always-listening systems:

```python
import numpy as np
import collections
import pyaudio
import webrtcvad
from scipy import signal

class WakeWordDetector:
    def __init__(self, wake_words=['robot', 'hey robot', 'assistant']):
        self.wake_words = wake_words
        self.vad = webrtcvad.Vad(3)  # Aggressive VAD
        
        # Audio parameters
        self.rate = 16000
        self.chunk_duration_ms = 30
        self.chunk_size = int(self.rate * self.chunk_duration_ms / 1000)
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        
        # Ring buffer for audio chunks
        self.ring_buffer = collections.deque(
            maxlen=30  # Store 30 chunks (900ms of audio)
        )
        
        # State
        self.is_awake = False
        self.listening_callback = None
    
    def set_listening_callback(self, callback):
        """Set callback to be called when wake word is detected"""
        self.listening_callback = callback
    
    def detect_wake_word(self, audio_chunk):
        """Detect if the audio chunk contains a wake word"""
        # This is a simplified implementation
        # In practice, you'd use a dedicated wake word detection model
        # like Porcupine, Snowboy, or a custom model
        
        # For demonstration, we'll simulate detection
        # by checking if the audio has speech-like characteristics
        if self.vad.is_speech(audio_chunk, self.rate):
            # Simulate successful detection after some time
            import random
            if random.random() < 0.1:  # 10% chance for demo
                return True
        return False
    
    def start_listening(self):
        """Start the wake word detection loop"""
        stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )
        
        try:
            while True:
                chunk = stream.read(self.chunk_size, exception_on_overflow=False)
                self.ring_buffer.append(chunk)
                
                if self.detect_wake_word(chunk):
                    if not self.is_awake:
                        self.is_awake = True
                        if self.listening_callback:
                            self.listening_callback()
                        
                        # Reset after wake word detection
                        self.ring_buffer.clear()
                        
                        # Wait for command (simplified)
                        import time
                        time.sleep(5)  # Wait 5 seconds for command
                        
                        self.is_awake = False
        except KeyboardInterrupt:
            pass
        finally:
            stream.stop_stream()
            stream.close()
```

### 2. Multi-Turn Dialogue Management

Handling complex conversations:

```python
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, Optional

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    WAITING_FOR_CONFIRMATION = "waiting_for_confirmation"
    EXECUTING = "executing"

@dataclass
class DialogueContext:
    state: DialogueState
    current_intent: Optional[str] = None
    entities: Dict[str, Any] = None
    waiting_for: Optional[str] = None
    conversation_id: Optional[str] = None

class DialogueManager:
    def __init__(self):
        self.context = DialogueContext(state=DialogueState.IDLE)
        self.command_parser = CommandParser()
        self.context_understanding = ContextualUnderstanding()
        
    def process_input(self, text: str) -> str:
        """Process user input and return system response"""
        if self.context.state == DialogueState.IDLE:
            return self.handle_initial_command(text)
        elif self.context.state == DialogueState.WAITING_FOR_CONFIRMATION:
            return self.handle_confirmation(text)
        else:
            # Handle other states
            return self.handle_follow_up(text)
    
    def handle_initial_command(self, text: str) -> str:
        """Handle the initial command from user"""
        command = self.command_parser.parse_command(text)
        
        if command:
            # Check if we have all required information
            missing_info = self.check_missing_info(command)
            
            if missing_info:
                # Ask for missing information
                self.context.state = DialogueState.WAITING_FOR_CONFIRMATION
                self.context.waiting_for = missing_info
                return f"Could you please specify {missing_info}?"
            else:
                # Execute the command
                self.context.state = DialogueState.PROCESSING
                return self.execute_command(command)
        else:
            return "I didn't understand that command. Could you rephrase it?"
    
    def check_missing_info(self, command) -> Optional[str]:
        """Check if the command has missing required information"""
        if command.action == 'move' and not command.location:
            return "where you'd like me to go"
        elif command.action == 'grasp' and not command.target:
            return "what you'd like me to pick up"
        elif command.action == 'place' and not command.target:
            return "what you'd like me to place"
        return None
    
    def handle_confirmation(self, text: str) -> str:
        """Handle user confirmation or additional information"""
        if self.context.waiting_for:
            # Parse the additional information
            # This is a simplified approach
            if 'to' in text or 'at' in text:
                # User provided location
                import re
                location_match = re.search(r'to (.+)|at (.+)', text.lower())
                if location_match:
                    location = location_match.group(1) or location_match.group(2)
                    # Update context and execute
                    self.context.state = DialogueState.PROCESSING
                    return f"Okay, going to {location}."
            elif 'the' in text or 'a' in text:
                # User specified an object
                # Simplified object extraction
                import re
                obj_match = re.search(r'(the |a |an )(.+)', text.lower())
                if obj_match:
                    obj = obj_match.group(2)
                    self.context.state = DialogueState.PROCESSING
                    return f"Okay, I'll get the {obj}."
        
        return "I'm not sure I understood. Could you clarify?"
    
    def handle_follow_up(self, text: str) -> str:
        """Handle follow-up questions or commands"""
        if 'yes' in text.lower() or 'yeah' in text.lower():
            # Confirm previous action
            return "Okay, proceeding with the action."
        elif 'no' in text.lower() or 'nope' in text.lower():
            # Cancel previous action
            self.context.state = DialogueState.IDLE
            return "Okay, I won't do that."
        else:
            # New command
            return self.handle_initial_command(text)
    
    def execute_command(self, command) -> str:
        """Execute the parsed command"""
        # This would interface with the action execution system
        if command.action == 'move':
            return f"Moving to {command.location}."
        elif command.action == 'grasp':
            return f"Attempting to grasp {command.target}."
        else:
            return f"Executing {command.action} action."
    
    def reset_dialogue(self):
        """Reset the dialogue context"""
        self.context = DialogueContext(state=DialogueState.IDLE)
```

Voice-to-Action systems are fundamental to creating natural, intuitive interactions between humans and humanoid robots. By enabling robots to understand and respond to spoken commands, these systems make robots more accessible and useful in everyday environments. The integration of robust speech recognition, natural language understanding, and action planning creates powerful capabilities for humanoid robots to assist humans in various tasks.