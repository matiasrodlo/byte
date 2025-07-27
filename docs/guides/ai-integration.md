# AI Integration Guide

Learn how to use Byte's advanced AI features for intelligent robot behavior.

## 🤖 AI Features Overview

Byte supports three types of AI integration:

1. **OpenAI GPT** - Cloud-based AI assistant
2. **Local LLM** - Offline AI processing
3. **LLM Server** - Custom AI server integration

## 🚀 OpenAI GPT Integration

### Setup

```bash
# Install OpenAI dependencies
pip install openai speech_recognition pyttsx3

# Set up API key
export OPENAI_API_KEY="your-api-key-here"
```

### Basic Usage

```python
from mind.gpt.gpt_dog import GPTDog

# Initialize GPT robot
gpt_dog = GPTDog()

# Start voice interaction
gpt_dog.start()
```

### Configuration

```python
from mind.gpt.gpt_dog import GPTDog

# Custom configuration
gpt_dog = GPTDog(
    api_key="your-api-key",
    model="gpt-4",
    assistant_name="Byte Assistant",
    voice_enabled=True,
    tts_enabled=True
)
```

### Voice Commands

```python
# Supported voice commands
commands = [
    "sit", "stand", "walk", "trot", "turn left", "turn right",
    "bark", "wag tail", "stretch", "pushup", "howling",
    "what's the weather?", "tell me a joke", "what time is it?"
]
```

## 🏠 Local LLM Integration

### Setup

```bash
# Install local LLM dependencies
pip install llama-cpp-python

# Download a GGUF model (e.g., TinyLlama)
wget https://huggingface.co/TheBloke/TinyLlama-1.1B-Chat-v1.0-GGUF/resolve/main/tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf
```

### Usage

```python
from mind.offline.local_llm_dog import LocalLLMDog

# Initialize local LLM robot
local_dog = LocalLLMDog(
    model_path="./tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf",
    n_ctx=2048,
    n_threads=4
)

# Start interaction
local_dog.start()
```

### Configuration Options

```python
local_dog = LocalLLMDog(
    model_path="./model.gguf",
    n_ctx=2048,           # Context window size
    n_threads=4,          # CPU threads
    n_gpu_layers=0,       # GPU layers (0 for CPU only)
    temperature=0.7,      # Response randomness
    max_tokens=512        # Maximum response length
)
```

## 🌐 LLM Server Integration

### Server Setup

```bash
# Start LLM server
cd mind/api
python app.py --model_path ./model.gguf --port 8000
```

### Client Usage

```python
from mind.api.robotdog_client import RobotDogClient

# Initialize client
client = RobotDogClient(
    server_url="http://localhost:8000",
    voice_enabled=True
)

# Start interaction
client.start()
```

### Server Configuration

```python
# Server options
python app.py \
    --model_path ./model.gguf \
    --port 8000 \
    --n_ctx 2048 \
    --n_threads 4 \
    --n_gpu_layers 0
```

## 🎯 Advanced AI Features

### Custom Actions

```python
from mind.gpt.action_flow import ActionFlow

# Define custom actions
custom_actions = {
    "dance": {
        "description": "Perform a dance routine",
        "function": lambda dog: [
            dog.do_action('wag_tail'),
            dog.do_action('turn_left'),
            dog.do_action('turn_right'),
            dog.do_action('bark')
        ]
    },
    "patrol": {
        "description": "Patrol the area",
        "function": lambda dog: [
            dog.do_action('walk', steps=5),
            dog.do_action('turn_left'),
            dog.do_action('walk', steps=5),
            dog.do_action('turn_right')
        ]
    }
}

# Add to action flow
action_flow = ActionFlow(custom_actions)
```

### Context Management

```python
# Maintain conversation context
context = {
    "user_name": "Alice",
    "robot_name": "Byte",
    "location": "living room",
    "time_of_day": "afternoon"
}

# Use in AI interactions
gpt_dog.set_context(context)
```

### Multi-Modal Input

```python
# Process multiple input types
def process_input(input_data):
    if isinstance(input_data, str):
        # Text input
        return process_text(input_data)
    elif hasattr(input_data, 'shape'):
        # Image input
        return process_image(input_data)
    elif hasattr(input_data, 'sample_rate'):
        # Audio input
        return process_audio(input_data)
```

## 🎮 Interactive Examples

### 1. Voice-Controlled Robot

```python
from mind.gpt.gpt_dog import GPTDog

def voice_control_demo():
    gpt_dog = GPTDog()
    
    print("Voice control demo started!")
    print("Say commands like:")
    print("- 'Walk forward'")
    print("- 'Turn left'")
    print("- 'Tell me a joke'")
    print("- 'What's the weather?'")
    
    try:
        gpt_dog.start()
    except KeyboardInterrupt:
        print("\nDemo ended")
    finally:
        gpt_dog.close()

if __name__ == "__main__":
    voice_control_demo()
```

### 2. Autonomous Behavior

```python
from mind.offline.local_llm_dog import LocalLLMDog

def autonomous_demo():
    local_dog = LocalLLMDog()
    
    print("Autonomous behavior demo")
    print("Robot will respond to environment automatically")
    
    try:
        while True:
            # Read sensors
            distance = local_dog.dog.ultrasonic.read()
            left_touch, right_touch = local_dog.dog.dual_touch.read()
            
            # Generate response based on environment
            if distance < 20:
                local_dog.process_input("Obstacle detected ahead")
            elif left_touch:
                local_dog.process_input("Left side touched")
            elif right_touch:
                local_dog.process_input("Right side touched")
            else:
                local_dog.process_input("Continue patrolling")
                
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nAutonomous demo ended")
    finally:
        local_dog.close()

if __name__ == "__main__":
    autonomous_demo()
```

### 3. Multi-Agent System

```python
from mind.api.robotdog_client import RobotDogClient
from mind.gpt.gpt_dog import GPTDog

def multi_agent_demo():
    # Initialize multiple AI systems
    gpt_dog = GPTDog()
    local_dog = LocalLLMDog()
    server_client = RobotDogClient()
    
    print("Multi-agent system demo")
    print("Using multiple AI systems for different tasks")
    
    try:
        # GPT for complex reasoning
        gpt_response = gpt_dog.process_input("Analyze the current situation")
        
        # Local LLM for quick responses
        local_response = local_dog.process_input("React to immediate environment")
        
        # Server for specialized tasks
        server_response = server_client.process_input("Perform specialized action")
        
        print(f"GPT: {gpt_response}")
        print(f"Local: {local_response}")
        print(f"Server: {server_response}")
        
    except Exception as e:
        print(f"Multi-agent demo error: {e}")
    finally:
        gpt_dog.close()
        local_dog.close()
        server_client.close()

if __name__ == "__main__":
    multi_agent_demo()
```

## 🔧 Configuration Files

### GPT Configuration

```python
# mind/gpt/keys.py
OPENAI_API_KEY = "your-api-key-here"
ASSISTANT_NAME = "Byte Assistant"
MODEL_NAME = "gpt-4"
TEMPERATURE = 0.7
MAX_TOKENS = 512
```

### Local LLM Configuration

```python
# mind/offline/keys.py
MODEL_PATH = "./model.gguf"
N_CTX = 2048
N_THREADS = 4
N_GPU_LAYERS = 0
TEMPERATURE = 0.7
```

### Server Configuration

```python
# mind/api/config.py
SERVER_HOST = "localhost"
SERVER_PORT = 8000
MODEL_PATH = "./model.gguf"
MAX_CONNECTIONS = 10
```

## 📊 Performance Optimization

### Model Selection

```python
# For Raspberry Pi 4 (4GB RAM)
model_config = {
    "model": "TinyLlama-1.1B-Chat-Q4_K_M.gguf",
    "n_ctx": 1024,
    "n_threads": 4,
    "n_gpu_layers": 0
}

# For more powerful systems
model_config = {
    "model": "Llama-2-7B-Chat-Q4_K_M.gguf",
    "n_ctx": 4096,
    "n_threads": 8,
    "n_gpu_layers": 32
}
```

### Memory Management

```python
# Optimize memory usage
import gc

def memory_optimized_inference():
    # Clear memory before inference
    gc.collect()
    
    # Process input
    response = model.generate(input_text)
    
    # Clear memory after inference
    gc.collect()
    
    return response
```

## 🐛 Troubleshooting

### Common Issues

**OpenAI API Errors**
```python
# Check API key
import os
print(f"API Key set: {bool(os.getenv('OPENAI_API_KEY'))}")

# Test connection
import openai
try:
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": "Hello"}]
    )
    print("API connection successful")
except Exception as e:
    print(f"API error: {e}")
```

**Local LLM Issues**
```python
# Check model file
import os
model_path = "./model.gguf"
if not os.path.exists(model_path):
    print(f"Model file not found: {model_path}")
    print("Download a GGUF model from Hugging Face")

# Check memory
import psutil
memory = psutil.virtual_memory()
print(f"Available memory: {memory.available / 1024**3:.1f} GB")
```

**Server Connection Issues**
```python
# Test server connection
import requests
try:
    response = requests.get("http://localhost:8000/health")
    print("Server is running")
except requests.exceptions.ConnectionError:
    print("Server is not running")
```

## 📚 Next Steps

- **[GPT Examples](../examples/gpt-examples.md)** - OpenAI integration examples
- **[Local LLM Examples](../examples/local-llm-examples.md)** - Offline AI examples
- **[Server Examples](../examples/server-examples.md)** - Custom server examples
- **[API Reference](../api/ai.md)** - Complete AI API documentation 