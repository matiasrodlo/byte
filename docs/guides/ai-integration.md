# AI Integration

## Overview

Byte supports three AI integration methods:

1. **GPT Integration** (`mind/gpt/`) - OpenAI GPT via API
2. **Local LLM** (`mind/offline/`) - Offline LLM using Ollama
3. **API Server** (`mind/api/`) - Custom LLM server integration

## GPT Integration

Uses OpenAI API for natural language processing.

```python
from mind.gpt.gpt_dog import GPTDog

# Initialize with API key
gpt_dog = GPTDog(api_key="your-key")

# Start voice interaction
gpt_dog.start()
```

**How it works:**
- Speech → Text (STT)
- Text → GPT API
- GPT response → Actions
- Response → Speech (TTS)

## Local LLM Integration

Uses Ollama for offline AI processing.

```python
from mind.offline.local_llm_dog import LocalLLMDog

# Initialize local LLM
llm_dog = LocalLLMDog()

# Start interaction
llm_dog.start()
```

**How it works:**
- Runs LLM locally on Raspberry Pi
- No internet required
- Uses Ollama API
- Converts LLM output to Byte actions

## API Server Integration

Connect to custom LLM server.

```python
from mind.api.robotdog_client import RobotDogClient

# Connect to server
client = RobotDogClient(server_url="http://localhost:8000")

# Send commands
response = client.send_command("walk forward")
```

**How it works:**
- HTTP requests to LLM server
- Server returns action JSON
- Client executes actions on Byte

## Action Flow

All AI integrations use `action_flow.py` to convert text to Byte actions:

```python
# Text input
"walk forward 5 steps"

# Parsed to action
{
    "action": "walk",
    "params": {"steps": 5}
}

# Executed on Byte
dog.do_action('walk', steps=5)
```

## Configuration

Each AI method has configuration files:
- `mind/gpt/keys.py` - OpenAI API key
- `mind/offline/keys.py` - Local LLM settings
- `mind/api/app.py` - Server configuration

See respective README files in `mind/` directories for setup.
