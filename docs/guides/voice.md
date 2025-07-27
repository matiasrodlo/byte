# Voice Commands

## How Voice Works

Byte uses speech recognition (STT) and text-to-speech (TTS) for voice interaction. STT converts microphone input to text, TTS converts text to audio output.

## Basic Voice Control

```python
from byte import RobotDog
import speech_recognition as sr

dog = RobotDog()
recognizer = sr.Recognizer()

while True:
    try:
        with sr.Microphone() as source:
            audio = recognizer.listen(source, timeout=5)
        
        # Convert speech to text
        command = recognizer.recognize_google(audio).lower()
        
        # Execute Byte action
        if 'sit' in command:
            dog.do_action('sit')
        elif 'stand' in command:
            dog.do_action('stand')
        elif 'walk' in command:
            dog.do_action('walk')
        elif 'quit' in command:
            break
    except sr.WaitTimeoutError:
        pass
    except sr.UnknownValueError:
        pass

dog.close()
```

## Text-to-Speech

Byte can speak using TTS engines.

```python
from byte import RobotDog
import pyttsx3

dog = RobotDog()
tts = pyttsx3.init()

# Byte speaks
tts.say("Hello, I am Byte")
tts.runAndWait()

dog.close()
```

## Voice Command Mapping

```python
VOICE_COMMANDS = {
    'sit': 'sit',
    'stand': 'stand',
    'walk': 'walk',
    'trot': 'trot',
    'turn left': 'turn_left',
    'turn right': 'turn_right',
    'bark': 'bark',
    'wag tail': 'wag_tail',
}

def execute_voice_command(dog, command):
    """Execute voice command on Byte."""
    command = command.lower()
    for voice, action in VOICE_COMMANDS.items():
        if voice in command:
            dog.do_action(action)
            return True
    return False
```

## AI Voice Integration

Byte supports AI voice assistants via `mind/gpt/` and `mind/offline/` modules.

```python
# GPT integration
from mind.gpt.gpt_dog import GPTDog
gpt_dog = GPTDog()
gpt_dog.start()

# Local LLM integration
from mind.offline.local_llm_dog import LocalLLMDog
llm_dog = LocalLLMDog()
llm_dog.start()
```

See [AI Integration Guide](ai-integration.md) for details.
