# Voice Commands Guide

Learn how to use Byte's voice recognition and text-to-speech capabilities.

## 🎤 Voice Features Overview

Byte supports:
- **Speech Recognition** - Convert speech to text
- **Text-to-Speech** - Convert text to speech
- **Voice Commands** - Control robot with voice
- **AI Voice Integration** - Natural language processing

## 🚀 Basic Voice Control

### Simple Voice Commands

```python
from byte import RobotDog
import speech_recognition as sr

def basic_voice_control():
    """Basic voice command system."""
    recognizer = sr.Recognizer()
    
    with RobotDog() as dog:
        print("Voice control active... Say commands like 'sit', 'stand', 'walk'")
        
        while True:
            try:
                with sr.Microphone() as source:
                    print("Listening...")
                    audio = recognizer.listen(source, timeout=5)
                
                # Convert speech to text
                command = recognizer.recognize_google(audio).lower()
                print(f"Heard: {command}")
                
                # Execute command
                if 'sit' in command:
                    dog.do_action('sit')
                elif 'stand' in command:
                    dog.do_action('stand')
                elif 'walk' in command:
                    dog.do_action('walk')
                elif 'bark' in command:
                    dog.do_action('bark')
                elif 'quit' in command:
                    break
                else:
                    print("Unknown command")
                    
            except sr.WaitTimeoutError:
                print("No speech detected")
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")

if __name__ == "__main__":
    basic_voice_control()
```

### Voice Command Dictionary

```python
# Predefined voice commands
VOICE_COMMANDS = {
    # Movement commands
    'sit': 'sit',
    'stand': 'stand',
    'lie down': 'lie',
    'walk': 'walk',
    'trot': 'trot',
    'turn left': 'turn_left',
    'turn right': 'turn_right',
    'stop': 'stop',
    
    # Behavior commands
    'bark': 'bark',
    'wag tail': 'wag_tail',
    'stretch': 'stretch',
    'pushup': 'pushup',
    'howl': 'howling',
    
    # LED commands
    'red light': lambda dog: dog.rgb_strip.set_color(255, 0, 0),
    'green light': lambda dog: dog.rgb_strip.set_color(0, 255, 0),
    'blue light': lambda dog: dog.rgb_strip.set_color(0, 0, 255),
    'lights off': lambda dog: dog.rgb_strip.off(),
    
    # System commands
    'quit': 'quit',
    'stop listening': 'quit',
    'calibrate': 'calibrate_servos'
}
```

## 🗣️ Text-to-Speech

### Basic TTS

```python
import pyttsx3

def setup_tts():
    """Setup text-to-speech engine."""
    engine = pyttsx3.init()
    
    # Configure voice
    voices = engine.getProperty('voices')
    if voices:
        engine.setProperty('voice', voices[0].id)  # Use first available voice
    
    # Configure speech rate
    engine.setProperty('rate', 150)  # Words per minute
    
    return engine

def speak_response(dog, text):
    """Speak text response."""
    engine = setup_tts()
    engine.say(text)
    engine.runAndWait()
```

### Interactive Voice System

```python
def interactive_voice_system():
    """Interactive voice control with TTS feedback."""
    recognizer = sr.Recognizer()
    tts_engine = setup_tts()
    
    with RobotDog() as dog:
        speak_response(dog, "Hello! I'm Byte. How can I help you?")
        
        while True:
            try:
                with sr.Microphone() as source:
                    print("Listening...")
                    audio = recognizer.listen(source, timeout=5)
                
                command = recognizer.recognize_google(audio).lower()
                print(f"Heard: {command}")
                
                # Process command
                if 'sit' in command:
                    dog.do_action('sit')
                    speak_response(dog, "Sitting down")
                elif 'stand' in command:
                    dog.do_action('stand')
                    speak_response(dog, "Standing up")
                elif 'walk' in command:
                    dog.do_action('walk', steps=5)
                    speak_response(dog, "Walking forward")
                elif 'how are you' in command:
                    speak_response(dog, "I'm doing great! Ready to play!")
                elif 'what can you do' in command:
                    speak_response(dog, "I can walk, sit, bark, and respond to your voice commands!")
                elif 'quit' in command:
                    speak_response(dog, "Goodbye!")
                    break
                else:
                    speak_response(dog, "I didn't understand that command")
                    
            except sr.WaitTimeoutError:
                print("No speech detected")
            except sr.UnknownValueError:
                print("Could not understand audio")
            except Exception as e:
                print(f"Error: {e}")
```

## 🎯 Advanced Voice Features

### Natural Language Processing

```python
import re

def parse_natural_language(command):
    """Parse natural language commands."""
    command = command.lower()
    
    # Movement patterns
    if re.search(r'walk\s+(\d+)\s+steps?', command):
        steps = int(re.search(r'walk\s+(\d+)\s+steps?', command).group(1))
        return ('walk', {'steps': steps})
    
    if re.search(r'turn\s+(\w+)\s+(\d+)\s+degrees?', command):
        direction = re.search(r'turn\s+(\w+)\s+(\d+)\s+degrees?', command).group(1)
        angle = int(re.search(r'turn\s+(\w+)\s+(\d+)\s+degrees?', command).group(2))
        return (f'turn_{direction}', {'angle': angle})
    
    # Behavior patterns
    if 'bark' in command:
        return ('bark', {})
    if 'wag' in command and 'tail' in command:
        return ('wag_tail', {})
    
    return None

def natural_language_control():
    """Control robot with natural language."""
    recognizer = sr.Recognizer()
    
    with RobotDog() as dog:
        print("Natural language control active...")
        print("Try: 'Walk 10 steps', 'Turn left 90 degrees', 'Bark three times'")
        
        while True:
            try:
                with sr.Microphone() as source:
                    print("Listening...")
                    audio = recognizer.listen(source, timeout=5)
                
                command = recognizer.recognize_google(audio)
                print(f"Heard: {command}")
                
                # Parse command
                parsed = parse_natural_language(command)
                if parsed:
                    action, params = parsed
                    dog.do_action(action, **params)
                    print(f"Executed: {action} with {params}")
                else:
                    print("Could not parse command")
                    
            except sr.WaitTimeoutError:
                print("No speech detected")
            except sr.UnknownValueError:
                print("Could not understand audio")
```

### Voice Training

```python
def voice_training():
    """Train voice recognition for better accuracy."""
    recognizer = sr.Recognizer()
    
    # Adjust for ambient noise
    with sr.Microphone() as source:
        print("Adjusting for ambient noise... Please wait...")
        recognizer.adjust_for_ambient_noise(source, duration=2)
        print("Ambient noise adjustment complete!")
    
    # Test recognition
    print("Testing voice recognition...")
    print("Say 'test' to verify recognition is working...")
    
    try:
        with sr.Microphone() as source:
            audio = recognizer.listen(source, timeout=5)
        
        result = recognizer.recognize_google(audio).lower()
        if 'test' in result:
            print("✅ Voice recognition working!")
        else:
            print(f"❌ Expected 'test', heard '{result}'")
            
    except Exception as e:
        print(f"❌ Voice recognition test failed: {e}")
```

## 🎮 Voice Game Examples

### Voice-Controlled Pet

```python
def voice_pet_simulation():
    """Simulate a voice-controlled pet."""
    recognizer = sr.Recognizer()
    tts_engine = setup_tts()
    
    with RobotDog() as dog:
        speak_response(dog, "Hi! I'm your voice-controlled pet!")
        
        while True:
            try:
                with sr.Microphone() as source:
                    print("Listening...")
                    audio = recognizer.listen(source, timeout=5)
                
                command = recognizer.recognize_google(audio).lower()
                print(f"Heard: {command}")
                
                # Pet-like responses
                if 'come here' in command or 'come' in command:
                    speak_response(dog, "Coming to you!")
                    dog.do_action('walk', steps=3)
                    
                elif 'good boy' in command or 'good girl' in command:
                    speak_response(dog, "Thank you! I'm happy!")
                    dog.do_action('wag_tail')
                    dog.do_action('bark')
                    
                elif 'play' in command:
                    speak_response(dog, "Let's play!")
                    dog.do_action('stretch')
                    dog.do_action('bark')
                    
                elif 'sleep' in command or 'bed' in command:
                    speak_response(dog, "Time for a nap...")
                    dog.do_action('lie')
                    dog.rgb_strip.set_color(0, 0, 50)  # Dim blue
                    
                elif 'wake up' in command:
                    speak_response(dog, "I'm awake!")
                    dog.do_action('stand')
                    dog.rgb_strip.off()
                    
                elif 'quit' in command:
                    speak_response(dog, "Goodbye! See you later!")
                    break
                    
            except sr.WaitTimeoutError:
                print("No speech detected")
            except sr.UnknownValueError:
                print("Could not understand audio")
```

### Voice-Controlled Assistant

```python
def voice_assistant():
    """Voice-controlled robot assistant."""
    recognizer = sr.Recognizer()
    tts_engine = setup_tts()
    
    with RobotDog() as dog:
        speak_response(dog, "Hello! I'm your robot assistant. How can I help?")
        
        while True:
            try:
                with sr.Microphone() as source:
                    print("Listening...")
                    audio = recognizer.listen(source, timeout=5)
                
                command = recognizer.recognize_google(audio).lower()
                print(f"Heard: {command}")
                
                # Assistant responses
                if 'patrol' in command:
                    speak_response(dog, "Starting patrol mode")
                    # Start patrol behavior
                    
                elif 'guard' in command:
                    speak_response(dog, "Guard mode activated")
                    dog.rgb_strip.set_color(255, 0, 0)  # Red for alert
                    
                elif 'status' in command:
                    distance = dog.ultrasonic.read()
                    speak_response(dog, f"Distance to nearest object is {distance} centimeters")
                    
                elif 'dance' in command:
                    speak_response(dog, "Let me show you a dance!")
                    # Perform dance routine
                    
                elif 'help' in command:
                    speak_response(dog, "I can patrol, guard, check status, dance, and respond to voice commands")
                    
                elif 'quit' in command:
                    speak_response(dog, "Assistant mode deactivated")
                    break
                    
            except sr.WaitTimeoutError:
                print("No speech detected")
            except sr.UnknownValueError:
                print("Could not understand audio")
```

## 🔧 Voice Configuration

### Microphone Setup

```python
def setup_microphone():
    """Setup and test microphone."""
    import pyaudio
    
    # List available microphones
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    
    print("Available microphones:")
    for i in range(0, numdevices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        if device_info.get('maxInputChannels') > 0:
            print(f"  {i}: {device_info.get('name')}")
    
    p.terminate()
```

### Voice Recognition Settings

```python
def configure_voice_recognition():
    """Configure voice recognition parameters."""
    recognizer = sr.Recognizer()
    
    # Adjust recognition parameters
    recognizer.energy_threshold = 4000  # Minimum audio energy
    recognizer.dynamic_energy_threshold = True  # Auto-adjust
    recognizer.pause_threshold = 0.8  # Pause detection
    recognizer.phrase_threshold = 0.3  # Phrase detection
    recognizer.non_speaking_duration = 0.5  # Non-speaking duration
    
    return recognizer
```

## 🐛 Troubleshooting

### Common Voice Issues

**Microphone not working:**
```python
# Check microphone permissions
# Test with different microphone
# Verify audio drivers
```

**Poor recognition accuracy:**
```python
# Adjust ambient noise
recognizer.adjust_for_ambient_noise(source, duration=2)

# Increase energy threshold
recognizer.energy_threshold = 5000

# Use better microphone
```

**TTS not working:**
```python
# Install TTS dependencies
pip install pyttsx3

# Check audio output
# Verify system audio
```

## 📚 Next Steps

- **[AI Integration](ai-integration.md)** - AI-powered voice processing
- **[Sensor Integration](sensors.md)** - Voice with sensor feedback
- **[Examples](../examples/voice.md)** - More voice examples
- **[API Reference](../api/voice.md)** - Complete voice API documentation 