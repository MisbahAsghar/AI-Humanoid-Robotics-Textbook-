# Conversational Robotics

## Purpose
Guide implementation of speech-to-text, LLM reasoning, and text-to-speech pipelines for natural voice-controlled robots with context-aware responses.

## When to Use
- Adding voice control to robot systems
- Implementing wake word detection and ASR
- Designing LLM prompts for robot task planning
- Managing conversation history and context
- Minimizing latency for real-time interaction

## Book Chapters
- Chapter 10: Conversational AI for Humanoid Robots

## Example Usage
```
User: "Set up a voice-controlled robot assistant with low latency"

Claude: Using the conversational-robotics skill, Chapter 10 shows the complete pipeline:

1. Wake word detection (Chapter 10, Section 1.1):
   - Porcupine for "Hey Robot" trigger
   - ~10ms latency, runs on CPU

2. Speech-to-text (Chapter 10, Section 1.2):
   - Whisper Tiny: 200ms latency, offline
   - Output: "Pick up the red cup"

3. LLM reasoning (Chapter 10, Section 1.3):
   - Local Llama 3 8B: 1-2s latency
   - Prompt: system context + conversation history + user input
   - Output: JSON action plan

4. Text-to-speech (Chapter 10, Section 1.4):
   - Piper TTS: 50-100ms latency, offline
   - Output: "I'll pick up the red cup now"

Total latency: ~2.5s (acceptable for manipulation tasks)

See Chapter 10, Section 1.5 for optimization strategies.
```
