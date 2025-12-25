# Reinforcement Learning

## Purpose
Guide implementation of RL algorithms (PPO, SAC) for robot control, including state/action design, reward shaping, and sim-to-real transfer with domain randomization.

## When to Use
- Training robot policies from scratch without demonstrations
- Optimizing control strategies through trial and error
- Designing observation and action spaces for tasks
- Implementing domain randomization for robustness
- Bridging the reality gap between simulation and hardware

## Book Chapters
- Chapter 8: Reinforcement Learning for Physical AI

## Example Usage
```
User: "What observation space should I use for a robotic arm reaching task?"

Claude: Using the reinforcement-learning skill, Chapter 8 Section 1.3.3 recommends:

State space for 7-DOF arm reaching:
1. Joint positions: [θ₁, θ₂, ..., θ₇] (7 values)
2. Joint velocities: [θ̇₁, θ̇₂, ..., θ̇₇] (7 values)
3. End-effector position: [x, y, z] (3 values)
4. Goal position: [x_goal, y_goal, z_goal] (3 values)
5. Distance to goal: ||p_ee - p_goal|| (1 value)

Total: 21-dimensional observation space

Action space: 7 joint torques [-1, 1]⁷, scaled to actuator limits

See Chapter 8, Section 1.3 for complete reaching example with PPO.
```
