# text_to_speech

Text to speech for ROS2 with multiple backend implementations:
- piper
- gtts

## Installation
```bash
pip install -r requirements.txt
```

## Usage

### Launch
```bash
ros2 launch text_to_speech bringup.launch.py 
```

### Execute action
```bash
ros2 action send_goal /say voice_interaction_msgs/action/Say "{'text': 'simple test', 'devices': [], 'voice_config': ''}" 
```