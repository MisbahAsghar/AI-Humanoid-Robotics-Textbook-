# Chapter 10 Code Examples

**Chapter**: Conversational Robotics
**Purpose**: Speech recognition, dialogue management, voice-controlled robots

---

## Requirements

- ROS 2 Humble
- Microphone + speakers
- OpenAI Whisper: `pip install openai-whisper pyaudio`
- OpenAI API key OR Ollama (local)
- espeak TTS: `sudo apt install espeak`

---

## Example Scripts

### `whisper_ros_node.py` (~100 lines)
- Continuous speech recognition with Whisper
- Voice Activity Detection (VAD) - simple amplitude threshold
- Publishes transcribed text to `/speech_text`
- Models: tiny/base/small/medium/large

### `dialogue_manager.py` (~90 lines)
- LLM-based conversation with history tracking
- Safety rules in system prompt
- JSON output: {message, action}
- Publishes to `/robot_response` and `/planned_action`

### `tts_node.py` (~35 lines)
- Text-to-speech using espeak
- Subscribes to `/robot_response`
- Plays audio through speakers

---

## Running

```bash
# Install dependencies
pip install openai-whisper pyaudio
sudo apt install espeak portaudio19-dev

# Run nodes
ros2 run conversational_robot whisper_asr
ros2 run conversational_robot dialogue_manager
ros2 run conversational_robot tts_node
```

---

**Last Updated**: 2025-12-23
