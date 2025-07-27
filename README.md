# Byte

A Raspberry Pi–powered robotic dog with intelligent interaction, emotional expression, and agile movement.

<img width="1200" height="801" alt="Byte Robot Dog" src="https://github.com/user-attachments/assets/a75ad140-43c3-42d9-b60d-fe0a085a250a" />

## Features

- **Facial Recognition** - Detects and responds to faces
- **Voice Interaction** - Understands spoken commands
- **Touch Responsiveness** - Reacts to capacitive sensors
- **Emotion Display** - Expresses through lights and sound
- **Obstacle Avoidance** - Navigates safely with ultrasonic sensors
- **Lifelike Movement** - 12 precision servos for natural motion

## Quick Start

```bash
# Install the package
pip install byte

# Run a demo
byte --demo

# Or use in Python
from byte import RobotDog
my_dog = RobotDog()
```

## Hardware

| Component | Specification |
|-----------|---------------|
| Processor | Raspberry Pi 4 |
| Camera | 5MP HD camera module |
| Servos | 12× metal gear servos |
| Sensors | Ultrasonic, touch, IMU, sound direction |
| Display | RGB light panel |
| Battery | 2× 18650 cells (1.5h runtime) |
| Body | Aluminum alloy |

## Examples

```python
from byte import RobotDog

# Initialize robot
dog = RobotDog()

# Basic movements
dog.do_action('sit')
dog.do_action('walk')
dog.do_action('bark')

# Sensor reading
distance = dog.ultrasonic.read()
touch = dog.dual_touch.read()

# Cleanup
dog.close()
```

## Documentation

- **Core API**: `byte/` - Main robot control
- **Examples**: `samples/` - Usage examples and tests
- **AI Integration**: `mind/` - GPT, local LLM, and API integration
- **System Scripts**: `bin/` - Installation and management

## License

MIT License - see LICENSE file for details.
