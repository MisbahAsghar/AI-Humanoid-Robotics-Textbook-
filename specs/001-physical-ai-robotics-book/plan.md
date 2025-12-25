# Implementation Plan: Physical AI and Humanoid Robotics Textbook

**Branch**: `001-physical-ai-robotics-book` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-robotics-book/spec.md`

## Summary

Create a comprehensive educational textbook on Physical AI and Humanoid Robotics using a **50/50 theory-practice balance** and **simulation-first pedagogy with optional hardware bridging**. The book covers foundations of embodied intelligence, ROS 2 middleware, simulation (Gazebo/Unity), NVIDIA Isaac ecosystem, Vision-Language-Action systems, and culminates in an autonomous humanoid capstone project. Target audience: CS students, AI/robotics engineers, and advanced learners building Physical AI systems.

**Technical Approach**: Docusaurus-based markdown book with tested Python code examples, architecture diagrams, hands-on exercises, troubleshooting guidance, and optional hardware deployment sections.

## Technical Context

**Language/Version**: Markdown/MDX (Docusaurus 2.x), Python 3.8+, ROS 2 Humble
**Primary Dependencies**: Docusaurus 2.x, ROS 2 Humble, Gazebo Classic/Sim, NVIDIA Isaac Sim (optional), Python robotics libraries (rclpy, tf2, etc.)
**Storage**: Static site generation (no database); GitHub Pages hosting
**Testing**: Manual validation of code examples, Docusaurus build tests, link checking, plagiarism checks
**Target Platform**: Web browsers (desktop + mobile), offline-readable documentation
**Project Type**: Educational content (documentation project)
**Performance Goals**: Fast page loads (<2s), mobile-responsive, offline capability
**Constraints**:
- All code examples must execute on Ubuntu 22.04 + ROS 2 Humble + Gazebo
- 50% theory / 50% practice content volume per chapter
- Zero broken links on publication
- Accessible (WCAG 2.1 AA - alt text for diagrams)
**Scale/Scope**:
- 10 user stories (foundations through capstone)
- ~8-12 chapters organized in 6 parts
- Est. 300-500 pages equivalent
- 50+ code examples
- 30+ diagrams

## Constitution Check

*GATE: Must pass before content creation. Re-check during chapter writing.*

### From Project Constitution

✅ **Technical Accuracy & Source Authority**:
- All claims cited from authoritative sources
- Tiered citation approach: peer-reviewed for foundations, preprints acceptable for emerging topics
- Sources labeled as "established" or "emerging/experimental"

✅ **Spec-Driven Content Development**:
- Following specify → clarify → plan → tasks → implement workflow
- Each chapter defined with learning objectives before writing

✅ **Clarity & Accessibility (NON-NEGOTIABLE)**:
- Target audience: CS/AI/Robotics students and engineers
- No marketing language, consistent terminology
- Progressive complexity with clear prerequisites

✅ **Engineering-First Mindset**:
- Emphasis on practical implementation (50% hands-on)
- Hardware-software integration covered
- Real-world constraints addressed

✅ **Ethical & Safety Awareness**:
- Safety considerations in Physical AI systems
- Ethical implications discussed where relevant

✅ **Docusaurus Standards & Structure**:
- All content in Markdown (.md or .mdx)
- Proper frontmatter, sidebar navigation
- GitHub Pages deployment-ready

### Additional Gates from Clarified Spec

✅ **50/50 Theory-Practice Balance**:
- Each major chapter includes conceptual explanation AND hands-on implementation
- Code examples tested and executable
- Troubleshooting guidance included

✅ **Simulation-First with Hardware Bridging**:
- Primary teaching in simulation (Gazebo/Isaac Sim)
- Optional "Deploying to Hardware" sections in major robotics chapters
- Sim-to-real transfer techniques covered

✅ **Tiered Citation Standards**:
- Foundational topics: peer-reviewed papers required
- Emerging topics: preprints from reputable labs (OpenAI, DeepMind, NVIDIA) acceptable
- All sources dated and labeled

## Book Structure

### High-Level Organization

The book is organized into **6 Parts** with **~12 chapters** following a progressive learning path from foundations to autonomous systems:

**Part 1: Foundations of Physical AI** (Chapters 1-2)
- Introduction to embodied intelligence
- Physical AI principles vs digital AI
- Sensor systems overview (LiDAR, cameras, IMUs, force/torque)

**Part 2: Robotic Nervous System - ROS 2** (Chapters 3-4)
- Nodes, Topics, Services, Actions architecture
- Python integration with ROS 2
- URDF basics for humanoid robot modeling

**Part 3: Digital Twin Simulation** (Chapters 5-6)
- Gazebo: physics, collisions, sensor simulation
- Unity (optional): visualization and digital twin creation

**Part 4: AI-Driven Perception and Control - NVIDIA Isaac** (Chapters 7-8)
- Isaac Sim / Isaac ROS / Isaac Lab overview
- VSLAM, navigation, perception pipelines
- Reinforcement learning for robot control

**Part 5: Vision-Language-Action Systems** (Chapters 9-10)
- Voice-to-Action integration (speech recognition + LLM planning)
- Cognitive planning: LLM → ROS 2 action translation
- Multi-modal human-robot interaction

**Part 6: Capstone Project** (Chapters 11-12)
- Autonomous humanoid: voice command → perception → manipulation → execution
- Simulation-first implementation guide
- Optional hardware deployment and troubleshooting
- Sim-to-real transfer techniques

### Detailed Chapter Outline

| Ch# | Title | Description | Theory/Practice Balance | Prerequisites | User Story |
|-----|-------|-------------|------------------------|---------------|------------|
| **Part 1: Foundations** | | | | | |
| 1 | Introduction to Physical AI | Embodied intelligence, Physical AI vs traditional AI, real-world constraints, sensor-motor loops | 70% theory / 30% examples | Python basics, basic AI/ML | US-1 (P1) |
| 2 | Humanoid Sensor Systems | RGB cameras, depth cameras, LiDAR, IMUs, force/torque sensors; sensor fusion basics | 60% theory / 40% sensor data examples | Chapter 1 | US-6 (P2) |
| **Part 2: ROS 2 Middleware** | | | | | |
| 3 | ROS 2 Fundamentals | Nodes, topics, services, actions; pub-sub architecture; Python rclpy examples | 40% theory / 60% practice | Python, Linux CLI | US-2 (P1) |
| 4 | URDF and Robot Modeling | URDF/SDF syntax, links, joints, sensors, actuators; humanoid robot description | 50% theory / 50% practice | Chapter 3 | US-3 (P1) |
| **Part 3: Simulation** | | | | | |
| 5 | Gazebo Simulation | Physics engines, collision models, world files, sensor plugins; launching simulations | 40% theory / 60% practice | Chapters 3-4 | US-3 (P1) |
| 6 | Unity Digital Twins (Optional) | Unity visualization, digital twin concepts, real-time visualization | 50% theory / 50% practice | Chapter 5 | US-3 (P1) |
| **Part 4: NVIDIA Isaac** | | | | | |
| 7 | Isaac Platform Overview | Isaac Sim, Isaac ROS, Isaac Lab; GPU-accelerated robotics; synthetic data generation | 60% theory / 40% setup/examples | Chapters 3-5 | US-4 (P2) |
| 8 | Perception and RL with Isaac | VSLAM, Nav2, object detection pipelines; RL training for locomotion; domain randomization | 40% theory / 60% practice | Chapter 7 | US-4, US-9 (P2-P3) |
| **Part 5: Vision-Language-Action** | | | | | |
| 9 | VLA Pipeline Architecture | Vision-Language-Action systems, LLM integration, task planning, action grounding | 60% theory / 40% architecture | Chapters 3, 6 | US-5 (P2) |
| 10 | Conversational Robotics | Speech recognition (Whisper), LLM-based planning, multi-modal interaction | 40% theory / 60% practice | Chapter 9 | US-5 (P2) |
| **Part 6: Capstone** | | | | | |
| 11 | Navigation and Manipulation | Nav2 stack, path planning (A*, RRT), inverse kinematics, grasp planning | 40% theory / 60% practice | Chapters 3-8 | US-7, US-8 (P3) |
| 12 | Autonomous Humanoid Capstone | Full integration: voice → perception → planning → manipulation → execution; sim-to-real | 30% theory / 70% practice | All chapters | US-10 (P3) |

### Chapter Structure Template

Each chapter follows this consistent structure to maintain 50/50 theory-practice balance:

```markdown
# Chapter N: [Title]

## Learning Objectives
- [Conceptual objective 1]
- [Practical skill objective 1]
- [Conceptual objective 2]
- [Practical skill objective 2]

## Prerequisites
- [Conceptual prerequisites]
- [Technical setup prerequisites]

## Part 1: Conceptual Foundations (Theory)
### 1.1 [Core Concept]
### 1.2 [System Architecture]
### 1.3 [Key Principles]

## Part 2: Hands-On Implementation (Practice)
### 2.1 Environment Setup
- Installation instructions
- Configuration steps
- Verification commands

### 2.2 Example 1: [Basic Implementation]
- Code walkthrough
- Expected output
- Common errors and troubleshooting

### 2.3 Example 2: [Advanced Implementation]
- Code walkthrough
- Expected output
- Common errors and troubleshooting

### 2.4 Exercises
- [Exercise 1 with solution guidance]
- [Exercise 2 with solution guidance]

## Part 3: Optional Hardware Deployment (if applicable)
### 3.1 Hardware Requirements
### 3.2 Deployment Steps
### 3.3 Sim-to-Real Considerations

## Review Questions
- [Question 1]
- [Question 2]
- [Question 3]

## Key Takeaways
- [Takeaway 1]
- [Takeaway 2]
- [Takeaway 3]

## References
- [Citation 1] (established/emerging)
- [Citation 2] (established/emerging)
```

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-robotics-book/
├── spec.md                 # Feature specification
├── plan.md                 # This file (architectural plan)
├── tasks.md                # Task breakdown (created by /sp.tasks)
├── checklists/
│   └── requirements.md     # Specification quality checklist
└── research/               # Research artifacts (citations, source evaluation)
    ├── foundational-sources.md
    ├── emerging-sources.md
    └── tool-documentation.md
```

### Source Code (Book Content)

```text
docs/
├── part-1-foundations/
│   ├── 01-introduction-physical-ai.md
│   └── 02-humanoid-sensor-systems.md
├── part-2-ros2/
│   ├── 03-ros2-fundamentals.md
│   └── 04-urdf-robot-modeling.md
├── part-3-simulation/
│   ├── 05-gazebo-simulation.md
│   └── 06-unity-digital-twins.md
├── part-4-isaac/
│   ├── 07-isaac-platform-overview.md
│   └── 08-perception-rl-isaac.md
├── part-5-vla/
│   ├── 09-vla-pipeline-architecture.md
│   └── 10-conversational-robotics.md
└── part-6-capstone/
    ├── 11-navigation-manipulation.md
    └── 12-autonomous-humanoid-capstone.md

static/
├── img/
│   ├── part-1/
│   ├── part-2/
│   ├── part-3/
│   ├── part-4/
│   ├── part-5/
│   └── part-6/
└── code-examples/
    ├── ros2-basics/
    ├── urdf-models/
    ├── gazebo-worlds/
    ├── isaac-configs/
    ├── vla-integration/
    └── capstone/

docusaurus.config.js        # Docusaurus configuration
sidebars.js                 # Navigation structure
package.json                # Dependencies
```

**Structure Decision**: Educational documentation project with Docusaurus static site generation. Content organized by learning progression (6 parts) rather than technical layers. Code examples stored in `/static/code-examples/` for easy download and testing. Diagrams organized by part for maintainability.

## Implementation Phases

### Phase 0: Research and Source Gathering (Week 1)

**Goal**: Establish authoritative source base and citation strategy

**Activities**:
1. **Foundational Sources (Peer-Reviewed)**:
   - Embodied intelligence: Brooks (1991), Pfeifer & Bongard (2006)
   - Physical AI principles: Recent survey papers on embodied AI
   - Robotics fundamentals: Siciliano & Khatib (2016 Springer Handbook)
   - SLAM algorithms: Durrant-Whyte & Bailey (2006), Cadena et al. (2016)

2. **Tool Documentation (Official Sources)**:
   - ROS 2 Humble documentation (docs.ros.org)
   - Gazebo documentation (gazebosim.org)
   - NVIDIA Isaac documentation (developer.nvidia.com/isaac)
   - Nav2 documentation (navigation.ros.org)

3. **Emerging Topics (Preprints + Industry)**:
   - VLA systems: Recent arXiv papers from Google DeepMind, OpenAI
   - Humanoid robotics: Tesla Optimus, Boston Dynamics, Figure AI publications
   - LLM-robot integration: Recent robotics + LLM papers (2024-2025)

**Deliverables**:
- `research/foundational-sources.md` - Peer-reviewed paper list with DOIs
- `research/emerging-sources.md` - Preprints and industry sources with dates
- `research/tool-documentation.md` - Official doc links with versions
- Citation template for Markdown: `[Author et al., Year](DOI/URL)` + label

**Success Criteria**:
- 50+ foundational sources identified and categorized
- All sources verified and accessible
- Citation labeling strategy defined (established vs emerging)

### Phase 1: Docusaurus Setup and Chapter Templates (Week 2)

**Goal**: Establish technical infrastructure and chapter templates

**Activities**:
1. **Docusaurus Initialization**:
   - Install Docusaurus 2.x
   - Configure `docusaurus.config.js` (site metadata, GitHub Pages deployment)
   - Set up `sidebars.js` with 6-part structure
   - Configure search plugin

2. **Chapter Template Creation**:
   - Create reusable MDX template following structure above
   - Define frontmatter schema (title, description, keywords, sidebar_position)
   - Create code block templates with language tags
   - Set up diagram placeholders with alt text templates

3. **CI/CD Pipeline**:
   - GitHub Actions workflow for automated builds
   - Link checking automation
   - Plagiarism check integration (Copyscape or similar)

4. **Style Guide**:
   - Markdown/MDX conventions
   - Code example formatting standards
   - Diagram style guidelines
   - Troubleshooting section template

**Deliverables**:
- Working Docusaurus site with navigation structure
- Chapter template file (`docs/_templates/chapter-template.mdx`)
- CI/CD pipeline configured
- Style guide document

**Success Criteria**:
- Docusaurus builds without errors
- Navigation structure reflects 6 parts and 12 chapters
- CI/CD successfully runs build and link check
- Template includes all required sections (learning objectives, theory, practice, review, references)

### Phase 2: Part 1 - Foundations (Weeks 3-4)

**Goal**: Complete Chapters 1-2 (Physical AI foundations, sensor systems)

**Chapter 1: Introduction to Physical AI**
- **Theory (70%)**:
  - Embodied intelligence concepts
  - Physical AI vs traditional AI comparison table
  - Real-world constraints (latency, power, noise)
  - Sensor-motor loop diagrams
- **Practice (30%)**:
  - Sensor data visualization examples (Python)
  - Simple sensor noise simulation
  - Physical constraint calculation examples
- **Sources**: Brooks (1991), Pfeifer & Bongard, recent Physical AI surveys
- **Diagrams**: 3-4 conceptual diagrams (embodied loop, constraint comparison)
- **Review**: 5 conceptual questions

**Chapter 2: Humanoid Sensor Systems**
- **Theory (60%)**:
  - Sensor types: RGB cameras, depth, LiDAR, IMU, force/torque
  - Sensor characteristics (FOV, resolution, noise)
  - Sensor fusion basics
- **Practice (40%)**:
  - Reading sensor data in Python
  - Visualizing point clouds (LiDAR)
  - IMU data interpretation
- **Sources**: Robotics Handbook chapters, sensor datasheets, ROS sensor tutorials
- **Diagrams**: 4-5 sensor diagrams, data flow charts
- **Review**: 5 questions + 2 hands-on exercises

**Deliverables**:
- Chapters 1-2 complete with all sections
- 5-8 diagrams created
- 8-10 code examples tested
- Citations complete (15-20 sources)

**Success Criteria** (per FR-011 to FR-014):
- Content volume: 50% theory / 50% practice (measure by page count or word count)
- All code examples execute successfully on Ubuntu 22.04 + Python 3.8+
- Troubleshooting sections include 3+ common errors
- Review questions and exercises included

### Phase 3: Part 2 - ROS 2 Middleware (Weeks 5-6)

**Goal**: Complete Chapters 3-4 (ROS 2 fundamentals, URDF modeling)

**Chapter 3: ROS 2 Fundamentals**
- **Theory (40%)**:
  - ROS 2 architecture overview
  - Nodes, topics, services, actions concepts
  - DDS middleware explanation
  - When to use topics vs services vs actions
- **Practice (60%)**:
  - Installing ROS 2 Humble
  - Creating a simple publisher/subscriber (Python)
  - Creating a service (Python)
  - Creating an action server/client (Python)
  - Visualizing node graphs with rqt
- **Sources**: ROS 2 Humble official docs, rclpy API docs
- **Diagrams**: 5-6 architecture diagrams (node graphs, communication patterns)
- **Review**: 5 questions + 3 hands-on exercises (create pub/sub, service, action)

**Chapter 4: URDF and Robot Modeling**
- **Theory (50%)**:
  - URDF/SDF syntax and structure
  - Links, joints, sensors, actuators
  - Coordinate frames and transforms
  - Humanoid robot anatomy
- **Practice (50%)**:
  - Writing a simple URDF file
  - Adding sensors to URDF
  - Visualizing URDF in RViz
  - Loading URDF in Gazebo
- **Sources**: URDF official docs, robotics textbooks (kinematics chapters)
- **Diagrams**: 6-7 diagrams (URDF structure, humanoid skeleton, coordinate frames)
- **Review**: 4 questions + 3 hands-on exercises (create URDF, add sensor, visualize)

**Deliverables**:
- Chapters 3-4 complete
- 11-13 diagrams
- 12-15 code examples (ROS 2 nodes, URDF files)
- Setup troubleshooting guide for ROS 2 installation

**Success Criteria**:
- 50% theory / 50% practice maintained
- All ROS 2 examples tested on Ubuntu 22.04 + ROS 2 Humble
- URDF examples load successfully in RViz and Gazebo
- Common installation errors documented with solutions

### Phase 4: Part 3 - Simulation (Weeks 7-8)

**Goal**: Complete Chapters 5-6 (Gazebo simulation, Unity digital twins)

**Chapter 5: Gazebo Simulation**
- **Theory (40%)**:
  - Physics engines (ODE, Bullet, DART)
  - Collision models and contact dynamics
  - Sensor simulation (camera, LiDAR, IMU)
  - World files and model SDFs
- **Practice (60%)**:
  - Installing Gazebo Classic or Gazebo Sim
  - Launching a world with humanoid robot
  - Spawning sensors in simulation
  - Reading simulated sensor data
  - Basic teleoperation
- **Optional Hardware Deployment**: Transferring URDF to real robot (conceptual)
- **Sources**: Gazebo official docs, Gazebo tutorials
- **Diagrams**: 5-6 diagrams (physics pipeline, sensor simulation, world structure)
- **Review**: 4 questions + 4 hands-on exercises (create world, spawn robot, read sensors, teleoperate)

**Chapter 6: Unity Digital Twins (Optional)**
- **Theory (50%)**:
  - Digital twin concepts
  - Unity for robotics visualization
  - Real-time vs recorded playback
- **Practice (50%)**:
  - Unity setup for robotics
  - Importing URDF into Unity
  - Visualizing robot state
  - ROS-Unity integration basics
- **Sources**: Unity Robotics Hub docs, digital twin papers
- **Diagrams**: 4-5 diagrams (digital twin architecture, Unity pipeline)
- **Review**: 3 questions + 2 hands-on exercises (import URDF, visualize state)

**Deliverables**:
- Chapters 5-6 complete
- 9-11 diagrams
- 10-12 code examples (Gazebo launch files, world files, Unity scripts)
- Sim-to-real considerations section in Chapter 5

**Success Criteria**:
- Gazebo examples work on Gazebo Classic and Gazebo Sim (note differences)
- Unity chapter marked as optional with simulation-only alternative
- Sim-to-real section discusses domain gap, physics fidelity, sensor noise

### Phase 5: Part 4 - NVIDIA Isaac (Weeks 9-11)

**Goal**: Complete Chapters 7-8 (Isaac platform, perception & RL)

**Chapter 7: Isaac Platform Overview**
- **Theory (60%)**:
  - Isaac Sim architecture
  - Isaac ROS for GPU-accelerated perception
  - Isaac Lab for RL training
  - Synthetic data generation concepts
  - Omniverse and USD format
- **Practice (40%)**:
  - Installing Isaac Sim (or using conceptual understanding if no GPU)
  - Loading humanoid in Isaac Sim
  - Configuring sensors
  - Basic Isaac ROS pipeline
- **Sources**: NVIDIA Isaac official docs, Isaac Sim tutorials, recent Isaac papers
- **Diagrams**: 7-8 diagrams (Isaac architecture, perception pipeline, RL workflow)
- **Review**: 5 questions + 2 hands-on exercises (load sim, configure sensors)

**Chapter 8: Perception and RL with Isaac**
- **Theory (40%)**:
  - VSLAM algorithms
  - Nav2 stack overview
  - Object detection and segmentation
  - Reinforcement learning for locomotion
  - Domain randomization and sim-to-real
- **Practice (60%)**:
  - Running VSLAM in Isaac Sim
  - Configuring Nav2 for navigation
  - Object detection with Isaac ROS
  - Training a simple RL policy (Isaac Lab)
  - Domain randomization parameters
- **Optional Hardware Deployment**: Nav2 on real robot, RL policy deployment
- **Sources**: SLAM papers (Cadena et al.), Nav2 docs, RL papers (PPO, SAC), domain randomization papers
- **Diagrams**: 8-10 diagrams (SLAM pipeline, Nav2 architecture, RL training loop, domain randomization)
- **Review**: 5 questions + 4 hands-on exercises (run VSLAM, configure Nav2, detect objects, train policy)

**Deliverables**:
- Chapters 7-8 complete
- 15-18 diagrams
- 12-15 code examples (Isaac configs, Nav2 params, RL training scripts)
- Hardware deployment sections for Nav2 and RL policies

**Success Criteria**:
- Isaac Sim examples include fallback for readers without GPU (conceptual walkthrough + expected outputs)
- RL training example runs in reasonable time (<1 hour on modest GPU)
- Domain randomization parameters documented with sim-to-real impact
- Hardware deployment sections marked as optional

### Phase 6: Part 5 - Vision-Language-Action (Weeks 12-13)

**Goal**: Complete Chapters 9-10 (VLA architecture, conversational robotics)

**Chapter 9: VLA Pipeline Architecture**
- **Theory (60%)**:
  - Vision-Language-Action system overview
  - LLM integration for task planning
  - Vision grounding (VLM object recognition)
  - Action primitive libraries
  - VLA system limitations and failure modes
- **Practice (40%)**:
  - Architecting a VLA system
  - LLM API integration (OpenAI or local Llama)
  - Action primitive definition
  - Simple task planning example
- **Sources**: Recent VLA papers (RT-1, RT-2, PaLM-E, etc.), LLM-robotics papers (2024-2025)
- **Diagrams**: 7-8 diagrams (VLA pipeline, task planning flow, action grounding)
- **Review**: 5 questions + 2 architecture exercises

**Chapter 10: Conversational Robotics**
- **Theory (40%)**:
  - Speech recognition (Whisper architecture)
  - LLM prompt engineering for robotics
  - Multi-modal interaction (voice + vision)
  - Safety constraints in LLM-driven actions
- **Practice (60%)**:
  - Integrating Whisper for speech recognition
  - LLM-based task planning implementation
  - ROS 2 action translation from LLM output
  - Voice-commanded robot demo (simulation)
- **Optional Hardware Deployment**: Running VLA on edge device (Jetson)
- **Sources**: Whisper paper (OpenAI), LLM-robotics integration tutorials, safety papers
- **Diagrams**: 6-7 diagrams (speech pipeline, LLM planning, safety constraints)
- **Review**: 4 questions + 3 hands-on exercises (integrate Whisper, LLM planning, voice command demo)

**Deliverables**:
- Chapters 9-10 complete
- 13-15 diagrams
- 10-12 code examples (VLA integration, LLM APIs, speech recognition, ROS action translation)
- Hardware deployment section for edge inference

**Success Criteria**:
- VLA examples use open-source alternatives (local Llama) alongside cloud APIs (OpenAI)
- LLM prompt templates provided for task planning
- Safety considerations explicitly documented (action validation, emergency stop)
- Voice command demo works in Gazebo simulation

### Phase 7: Part 6 - Capstone Project (Weeks 14-16)

**Goal**: Complete Chapters 11-12 (navigation/manipulation, autonomous humanoid)

**Chapter 11: Navigation and Manipulation**
- **Theory (40%)**:
  - Path planning algorithms (A*, RRT, DWA)
  - Costmaps and obstacle avoidance
  - Inverse kinematics (IK) for manipulation
  - Grasp planning strategies
  - Force control basics
- **Practice (60%)**:
  - Configuring Nav2 for autonomous navigation
  - Implementing waypoint navigation
  - IK solver setup (KDL or MoveIt)
  - Pick-and-place pipeline
  - Debugging navigation failures
- **Optional Hardware Deployment**: Nav2 tuning on real robot, real-world grasping
- **Sources**: Path planning papers, Nav2 docs, MoveIt docs, grasp planning papers
- **Diagrams**: 10-12 diagrams (path planning comparison, costmap layers, IK geometry, grasp pipeline)
- **Review**: 5 questions + 5 hands-on exercises (configure Nav2, waypoint nav, IK setup, pick-place, debug)

**Chapter 12: Autonomous Humanoid Capstone**
- **Theory (30%)**:
  - System integration architecture
  - State machine design for autonomous behavior
  - Sim-to-real transfer checklist
  - Failure handling and recovery
- **Practice (70%)**:
  - **Capstone Project Implementation**:
    1. Voice command input (Whisper)
    2. LLM task planning ("pick up blue cube")
    3. Perception (object detection in Isaac Sim)
    4. Navigation (Nav2 to object location)
    5. Manipulation (IK + grasp execution)
    6. Task completion feedback
  - Full system testing in simulation
  - Troubleshooting common integration issues
- **Optional Hardware Deployment**: Deploying capstone to real humanoid robot
- **Sources**: Integration architecture patterns, state machine design, previous chapters
- **Diagrams**: 8-10 diagrams (full system architecture, state machine, data flow, integration points)
- **Review**: 3 questions + 1 capstone project (implement autonomous humanoid)

**Deliverables**:
- Chapters 11-12 complete
- 18-22 diagrams
- 15-20 code examples (Nav2 configs, IK solvers, state machines, full integration)
- Complete capstone project code repository
- Troubleshooting guide for integration issues
- Sim-to-real transfer guide

**Success Criteria**:
- Capstone project runs successfully in Gazebo or Isaac Sim
- All components integrated (speech, LLM, perception, navigation, manipulation)
- At least one successful task completion demonstrated
- Hardware deployment guide included as optional extension
- Troubleshooting covers 10+ common integration failures

### Phase 8: Final Review and Deployment (Week 17)

**Goal**: Validate entire book, deploy to GitHub Pages

**Activities**:
1. **Quality Validation**:
   - Review all chapters for 50/50 theory-practice balance
   - Verify all code examples execute successfully
   - Check all citations and links (automated + manual)
   - Plagiarism check on all content
   - Peer review by technical experts

2. **Completeness Check** (against Success Criteria):
   - SC-001 to SC-010: Reader learning outcomes validated via exercises
   - SC-011: Zero broken links confirmed
   - SC-012: All foundational topics cite peer-reviewed sources
   - SC-013: Hardware deployment sections present in Chapters 5, 8, 10, 11, 12
   - SC-014: 50% hands-on content confirmed (measure by page/word count)
   - SC-015: All code tested on Ubuntu 22.04 + ROS 2 Humble + Gazebo
   - SC-016: Troubleshooting sections complete (3+ errors per hands-on section)
   - SC-017: Docusaurus builds without errors
   - SC-018: All diagrams have alt text
   - SC-019: Learning objectives state conceptual + practical skills
   - SC-020: Plagiarism check passed

3. **Deployment**:
   - Final Docusaurus build
   - GitHub Pages deployment
   - Mobile responsiveness testing
   - Search functionality testing
   - Offline access verification

**Deliverables**:
- Complete book (all 12 chapters)
- All diagrams, code examples, references complete
- Deployed to GitHub Pages
- Quality validation report

**Success Criteria**:
- All SC-001 to SC-027 pass
- Book builds and deploys successfully
- Zero broken links, all citations verified
- Peer reviewers confirm accuracy (4.5/5 or higher)

## Technical Decisions and Rationale

### 1. Docusaurus vs Other Platforms

**Decision**: Use Docusaurus 2.x for static site generation

**Rationale**:
- Built-in search, sidebar navigation, dark mode
- MDX support for interactive components
- GitHub Pages deployment built-in
- Offline-capable (PWA support)
- Active community and plugin ecosystem
- Versioning support for future editions

**Alternatives Considered**:
- GitBook: Less customizable, hosting costs
- MkDocs: Python-based, less modern UI
- Jupyter Book: Overly focused on notebooks

### 2. Simulation Platform Selection

**Decision**: Gazebo as primary, Isaac Sim as secondary/optional

**Rationale**:
- Gazebo: Open-source, widely adopted in ROS community, runs on modest hardware
- Isaac Sim: State-of-the-art but requires NVIDIA GPU, proprietary
- Both covered to maximize accessibility while showcasing modern tools

**Trade-offs**:
- Gazebo more accessible but less performant for large-scale RL
- Isaac Sim more powerful but hardware-gated
- Solution: Isaac chapters include conceptual walkthroughs for readers without GPU

### 3. Code Example Language

**Decision**: Python 3.8+ for all code examples

**Rationale**:
- Python is lingua franca of AI/ML community
- rclpy (ROS 2 Python) well-documented
- Easier for students than C++
- Compatible with LLM APIs (OpenAI, HuggingFace)

**Trade-offs**:
- Python slower than C++ for real-time control
- Solution: Note performance considerations, mention C++ as production alternative

### 4. LLM Integration Approach

**Decision**: Support both cloud APIs (OpenAI) and local models (Llama)

**Rationale**:
- Cloud APIs easier for beginners, better performance
- Local models important for privacy, offline use, cost
- Both approaches teach core integration concepts

**Implementation**:
- Provide code examples for both
- Default to OpenAI for simplicity, show Llama conversion
- Document cost and latency trade-offs

### 5. Hardware Deployment Coverage

**Decision**: Simulation-first with optional hardware bridging sections

**Rationale** (from clarified spec):
- Accessibility: Not all readers have humanoid robots
- Safety: Simulation safer for learning
- Cost: Simulation free vs expensive hardware
- Iteration speed: Simulation much faster

**Implementation**:
- Mark all hardware sections as "Optional"
- Provide simulation alternatives for all content
- Include dedicated sim-to-real chapter covering domain gap

## Risk Mitigation

### Risk 1: Rapid Tool Evolution (ROS versions, Isaac updates)

**Mitigation**:
- Pin specific versions (ROS 2 Humble, Isaac Sim 2024.x)
- Document all version-specific behavior
- Create "Compatibility Notes" callouts for newer versions
- Plan for annual content review and updates

### Risk 2: Hardware Unavailability for Optional Sections

**Mitigation**:
- All hardware sections marked as optional
- Simulation alternatives always provided
- Expected outputs documented so readers can follow along
- Video demonstrations linked where helpful

### Risk 3: 50/50 Balance Drift During Writing

**Mitigation**:
- Measure theory vs practice by page/word count per chapter
- Peer review checks balance explicitly (SC-022)
- Template enforces structure (Part 1: Theory, Part 2: Practice)
- Adjust during Phase 8 review if imbalance detected

### Risk 4: Code Examples Break with Updates

**Mitigation**:
- Pin dependencies in code example metadata
- Automated testing of code examples in CI/CD
- Document required environment precisely (Ubuntu 22.04, ROS 2 Humble, etc.)
- Include version compatibility notes

### Risk 5: Citation Link Rot

**Mitigation**:
- Prefer DOIs over URLs (DOIs persistent)
- Use Wayback Machine for important non-academic sources
- Automated link checking in CI/CD
- Plan for periodic link maintenance

### Risk 6: Scope Creep (Adding Non-Essential Content)

**Mitigation**:
- Strict adherence to spec user stories
- "Out of Scope" reminder in plan (no hardware assembly, no firmware, no vendor comparisons)
- Review against FR-001 to FR-035 before adding content
- Peer review catches scope violations

## Next Steps

### Immediate Next Step: `/sp.tasks`

Now that the architectural plan is complete, proceed to `/sp.tasks` to generate chapter-wise task breakdown:

**Expected Task Structure**:
- **Phase 0 Tasks**: Research and source gathering (10-15 tasks)
- **Phase 1 Tasks**: Docusaurus setup and templates (8-10 tasks)
- **Phase 2-7 Tasks**: Chapter writing (120-150 tasks, ~10-12 per chapter)
  - Each chapter broken into: outline, theory sections, practice sections, diagrams, code examples, review questions, references
- **Phase 8 Tasks**: Final review and deployment (15-20 tasks)

**Task Dependencies**:
- Research (Phase 0) must complete before any writing
- Docusaurus setup (Phase 1) must complete before content creation
- Chapters have linear dependencies: Part 1 → Part 2 → Part 3 → ... → Part 6
- Within each chapter: outline → content → review

### Follow-On Activities

After `/sp.tasks`:
1. **`/sp.implement`**: Execute task breakdown to write chapters
2. **Peer Review**: Technical experts review for accuracy (SC-021)
3. **User Testing**: Beta readers attempt exercises (SC-024)
4. **Deployment**: GitHub Pages publication
5. **Post-Publication**: Reader surveys (SC-023), content updates

## Appendix: Alignment with Specification

This plan directly addresses all 35 Functional Requirements:

**Content Coverage (FR-001 to FR-010)**: ✅ All topics covered across 12 chapters
**Theory-Practice Balance (FR-011 to FR-014)**: ✅ 50/50 enforced via chapter structure and measurement
**Simulation-to-Reality (FR-015 to FR-018)**: ✅ Simulation-first with optional hardware sections in Chapters 5, 8, 10, 11, 12
**Citation Standards (FR-019 to FR-024)**: ✅ Research phase establishes tiered citation approach
**Technical Structure (FR-025 to FR-035)**: ✅ Docusaurus structure, chapter templates, prerequisites defined

This plan also ensures all 27 Success Criteria are measurable and achievable through the defined phases and validation checkpoints.
