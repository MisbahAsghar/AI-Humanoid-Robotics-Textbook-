# Feature Specification: Physical AI and Humanoid Robotics Textbook

**Feature Branch**: `001-physical-ai-robotics-book`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Book on Physical AI and Humanoid Robotics"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Understanding Physical AI Foundations (Priority: P1)

A computer science student or AI engineer needs to understand how AI systems transition from purely digital environments to physical embodiment. They need to grasp the fundamental differences between traditional AI and Physical AI, including concepts of embodied intelligence, sensor-motor loops, and real-world constraints.

**Why this priority**: Foundation knowledge is essential before readers can understand any advanced topics. Without this, the entire book structure fails.

**Independent Test**: Reader can explain the difference between traditional AI and Physical AI, describe the role of embodiment in intelligence, and identify at least three unique challenges of Physical AI systems (e.g., latency, power constraints, sensor noise).

**Acceptance Scenarios**:

1. **Given** a reader with AI/ML background, **When** they complete the foundations chapter, **Then** they can articulate how embodiment changes the AI problem space
2. **Given** chapter completion, **When** asked to compare virtual assistant AI vs humanoid robot AI, **Then** they can identify hardware-software integration challenges
3. **Given** foundation knowledge, **When** presented with a Physical AI system design problem, **Then** they can identify relevant constraints (compute, latency, safety)

---

### User Story 2 - Understanding ROS 2 Architecture and Middleware (Priority: P1)

A robotics engineer or advanced CS student needs to understand how ROS 2 serves as the middleware backbone for humanoid robotics systems. They must grasp the publish-subscribe model, nodes, topics, services, and actions, and understand why this architecture matters for distributed robotic systems.

**Why this priority**: ROS 2 is the industry-standard middleware. Without understanding it, readers cannot comprehend how modern humanoid systems are architected or engage with the practical implementation examples throughout the book.

**Independent Test**: Reader can diagram a simple ROS 2 system with multiple nodes communicating via topics, explain when to use topics vs services vs actions, and write basic Python code to create a node that publishes sensor data.

**Acceptance Scenarios**:

1. **Given** ROS 2 chapter completion, **When** reader sees a humanoid system architecture diagram, **Then** they can identify communication patterns and data flows
2. **Given** understanding of ROS 2 primitives, **When** asked to design a perception-to-action pipeline, **Then** they can select appropriate communication mechanisms (topics for continuous data, services for request-response, actions for long-running tasks)
3. **Given** Python and ROS 2 knowledge, **When** provided with a robotics task (e.g., "stream camera data to a vision processor"), **Then** they can write the corresponding publisher/subscriber code structure

---

### User Story 3 - Understanding Simulation and Digital Twins (Priority: P1)

A robotics developer needs to understand simulation-first development methodology, digital twin concepts, and physics simulation for robotics. They must learn how to model robots using URDF/SDF, simulate them in Gazebo or Unity, and understand the trade-offs between simulation fidelity and compute cost.

**Why this priority**: Simulation is the foundation for safe, cost-effective development. Before deploying to physical hardware, all development happens in simulation. This is critical for the book's practical approach.

**Independent Test**: Reader can explain what a digital twin is, describe the purpose of URDF/SDF files, set up a basic simulated robot in Gazebo, and identify three differences between simulated and real-world robot behavior.

**Acceptance Scenarios**:

1. **Given** simulation chapter completion, **When** reader is asked why simulation matters, **Then** they can list at least three benefits (safety, cost, iteration speed, data generation)
2. **Given** understanding of robot modeling, **When** shown a URDF file, **Then** they can identify links, joints, sensors, and actuators
3. **Given** knowledge of simulation limitations, **When** designing a sim-to-real transfer strategy, **Then** they can identify domain gaps and propose mitigation strategies (domain randomization, system identification, etc.)

---

### User Story 4 - Understanding NVIDIA Isaac Ecosystem (Priority: P2)

An AI/robotics engineer working on Physical AI systems needs to understand NVIDIA's Isaac platform (Isaac Sim, Isaac ROS, Isaac Lab) and how it enables synthetic data generation, GPU-accelerated perception, and reinforcement learning for robotics.

**Why this priority**: Isaac represents state-of-the-art tooling for Physical AI development. Understanding it enables readers to leverage modern, industry-standard workflows.

**Independent Test**: Reader can describe the purpose of each Isaac component (Sim, ROS, Lab), explain how Isaac Sim generates synthetic training data, and outline how Isaac ROS accelerates perception pipelines using GPU compute.

**Acceptance Scenarios**:

1. **Given** Isaac chapter completion, **When** asked how to generate training data for a humanoid vision system, **Then** they can describe using Isaac Sim with domain randomization
2. **Given** understanding of Isaac ROS, **When** designing a real-time perception pipeline, **Then** they can explain how GPU acceleration reduces latency
3. **Given** Isaac Lab knowledge, **When** asked about training humanoid locomotion policies, **Then** they can describe the RL training workflow in simulation

---

### User Story 5 - Understanding Vision-Language-Action (VLA) Systems (Priority: P2)

A researcher or engineer building autonomous humanoid systems needs to understand how Vision-Language-Action models bridge perception, language understanding, and physical action. They must grasp how LLMs and VLMs are integrated into robotic control systems for task planning and execution.

**Why this priority**: VLA systems represent the cutting edge of humanoid AI, enabling natural language interaction and generalizable behavior. This is essential for autonomous humanoids.

**Independent Test**: Reader can explain the VLA pipeline (vision input → language model reasoning → action output), describe how LLMs enable high-level task planning, and identify limitations of current VLA approaches.

**Acceptance Scenarios**:

1. **Given** VLA chapter completion, **When** shown a humanoid receiving voice command "bring me the red cup," **Then** they can trace the processing pipeline from speech-to-text through vision grounding to manipulation
2. **Given** understanding of VLA limitations, **When** designing a humanoid system, **Then** they can identify scenarios where VLA excels vs where traditional approaches are needed
3. **Given** knowledge of VLA architectures, **When** asked to integrate an LLM into a robot control system, **Then** they can describe the system architecture and data flow

---

### User Story 6 - Understanding Humanoid Perception Systems (Priority: P2)

A robotics engineer needs to understand the sensor suite for humanoid robots (RGB cameras, depth cameras, LiDAR, IMUs, force/torque sensors) and the perception algorithms (object detection, pose estimation, SLAM, semantic segmentation) that enable environmental understanding.

**Why this priority**: Perception is how robots understand their environment. Without robust perception, no autonomous behavior is possible.

**Independent Test**: Reader can list the primary sensors on a humanoid robot and their purposes, explain SLAM and localization, and describe how vision models (YOLO, Mask R-CNN, etc.) are deployed for real-time object detection.

**Acceptance Scenarios**:

1. **Given** perception chapter completion, **When** asked how a humanoid navigates an unknown room, **Then** they can explain the SLAM process and sensor fusion
2. **Given** understanding of vision models, **When** asked to implement object detection for manipulation, **Then** they can select appropriate models and describe deployment considerations (latency, accuracy trade-offs)
3. **Given** knowledge of sensor types, **When** designing a humanoid for a specific task (e.g., warehouse navigation), **Then** they can specify required sensors and justify choices

---

### User Story 7 - Understanding Humanoid Navigation and Path Planning (Priority: P3)

A robotics developer needs to understand how humanoid robots navigate environments, including path planning algorithms (A*, RRT, DWA), obstacle avoidance, and the Nav2 stack in ROS 2.

**Why this priority**: Navigation is a core competency for mobile robots, but it builds on perception (P2) and ROS 2 (P1) knowledge.

**Independent Test**: Reader can explain the Nav2 architecture, describe the difference between global and local planning, and implement basic waypoint navigation using Nav2 APIs.

**Acceptance Scenarios**:

1. **Given** navigation chapter completion, **When** asked to implement autonomous navigation from point A to B, **Then** they can configure Nav2 and explain the role of costmaps
2. **Given** understanding of planning algorithms, **When** choosing between A* and RRT for a humanoid in cluttered environment, **Then** they can justify the choice based on environment characteristics
3. **Given** Nav2 knowledge, **When** debugging a navigation failure, **Then** they can interpret visualization data and identify issues (localization drift, costmap misconfiguration, etc.)

---

### User Story 8 - Understanding Manipulation and Control (Priority: P3)

An engineer designing humanoid manipulation systems needs to understand inverse kinematics, trajectory planning, grasp planning, and force control for robotic arms and hands.

**Why this priority**: Manipulation is critical for many humanoid tasks, but it's more specialized than navigation and builds on earlier concepts.

**Independent Test**: Reader can explain inverse kinematics, describe the manipulation pipeline (grasp detection → trajectory planning → execution), and identify common manipulation challenges (contact dynamics, grasp stability).

**Acceptance Scenarios**:

1. **Given** manipulation chapter completion, **When** asked to implement pick-and-place, **Then** they can outline the full pipeline including perception, grasp planning, and motion execution
2. **Given** IK understanding, **When** provided with a target end-effector pose, **Then** they can explain how joint angles are computed
3. **Given** force control knowledge, **When** designing a system for delicate object manipulation, **Then** they can describe force-torque sensor integration and control strategies

---

### User Story 9 - Understanding Reinforcement Learning for Humanoid Control (Priority: P3)

A researcher or advanced engineer needs to understand how reinforcement learning is used to train humanoid locomotion policies, manipulation skills, and whole-body control, including sim-to-real transfer techniques.

**Why this priority**: RL represents state-of-the-art for learning complex control policies, but requires all prior knowledge (simulation, sensors, control) to be understood first.

**Independent Test**: Reader can explain the RL training loop for humanoid locomotion, describe domain randomization and other sim-to-real techniques, and identify when to use RL vs traditional control approaches.

**Acceptance Scenarios**:

1. **Given** RL chapter completion, **When** asked to train a bipedal walking policy, **Then** they can describe the reward function design, training environment setup, and evaluation process
2. **Given** sim-to-real knowledge, **When** deploying a policy from simulation to real hardware, **Then** they can anticipate domain gap issues and apply mitigation strategies
3. **Given** understanding of RL limitations, **When** designing a humanoid system, **Then** they can decide when to use learned policies vs analytical control

---

### User Story 10 - Building an Autonomous Humanoid Agent (Capstone) (Priority: P3)

A reader who has completed all previous chapters can now integrate all concepts to design and simulate an autonomous humanoid agent that receives voice commands, interprets them using an LLM, perceives the environment, plans actions, and executes tasks.

**Why this priority**: This is the culminating project that ties together all book concepts. It validates that the reader has achieved comprehensive understanding.

**Independent Test**: Reader can design a system architecture for an autonomous humanoid, implement a simulated agent in Isaac Sim or Gazebo that responds to natural language commands, and demonstrate successful task completion (e.g., "pick up the blue cube and place it on the table").

**Acceptance Scenarios**:

1. **Given** completion of all chapters, **When** asked to design an autonomous humanoid system, **Then** they can produce a complete architecture diagram showing all components (perception, planning, control, language understanding, etc.)
2. **Given** all prerequisite knowledge, **When** implementing the capstone project, **Then** they successfully integrate speech recognition, LLM planning, vision perception, and manipulation control
3. **Given** a working simulated system, **When** asked about deployment to real hardware, **Then** they can identify required modifications and potential challenges

---

### Edge Cases

- What happens when the reader lacks Python programming experience? (Assumption: basic programming required; provide Python primer references)
- What happens when reader has no Linux/command-line experience? (Assumption: Linux familiarity expected; provide setup guide and CLI resources)
- How does the book handle readers interested only in specific topics (e.g., only VLA systems)? (Solution: modular chapter structure with clear prerequisites)
- What if cited sources become outdated or links break? (Mitigation: use DOIs where available, archive links, periodic content review)
- How to ensure technical accuracy across rapidly evolving field? (Solution: cite authoritative sources, date all claims, note emerging vs established techniques)

## Requirements *(mandatory)*

### Functional Requirements

#### Content Coverage
- **FR-001**: Book MUST provide comprehensive coverage of Physical AI foundations, including embodied intelligence, sensor-motor loops, and real-world constraints
- **FR-002**: Book MUST explain ROS 2 architecture including nodes, topics, services, actions, and provide tested, executable Python code examples
- **FR-003**: Book MUST cover robot modeling using URDF/SDF and simulation using Gazebo with hands-on exercises
- **FR-004**: Book MUST explain NVIDIA Isaac Sim, Isaac ROS, and Isaac Lab with practical use cases and setup guidance
- **FR-005**: Book MUST describe Vision-Language-Action pipeline architectures and integration with LLMs/VLMs
- **FR-006**: Book MUST cover humanoid sensor systems (cameras, LiDAR, IMUs, force/torque) and perception algorithms (SLAM, object detection, segmentation)
- **FR-007**: Book MUST explain navigation and path planning including Nav2 stack with configuration examples
- **FR-008**: Book MUST cover manipulation and control including inverse kinematics and grasp planning
- **FR-009**: Book MUST explain reinforcement learning for humanoid control and sim-to-real transfer
- **FR-010**: Book MUST include a capstone project demonstrating autonomous humanoid agent in simulation with implementation guide

#### Theory-Practice Balance (CLARIFIED)
- **FR-011**: Each major topic chapter MUST include both conceptual explanation (theory) and hands-on implementation (practice) in approximately 50/50 balance
- **FR-012**: Code examples MUST be tested, executable, and include setup/installation guidance where needed
- **FR-013**: Each hands-on section MUST include expected outputs, common errors, and troubleshooting guidance
- **FR-014**: Book MUST include exercises or review questions at chapter end to reinforce learning

#### Simulation-to-Reality Coverage (CLARIFIED)
- **FR-015**: Primary content MUST focus on simulation-based development with Gazebo and/or Isaac Sim
- **FR-016**: Each major robotics topic (perception, navigation, manipulation) MUST include optional "Deploying to Hardware" section discussing real-world considerations
- **FR-017**: Book MUST include dedicated chapter or sections on sim-to-real transfer techniques (domain randomization, system identification, reality gap)
- **FR-018**: Physical hardware deployment guidance MUST clearly mark as optional and provide simulation alternatives

#### Citation and Source Standards (CLARIFIED)
- **FR-019**: Foundational topics (Physical AI principles, established algorithms) MUST cite peer-reviewed papers
- **FR-020**: Emerging topics (VLA systems, latest tools) MAY cite preprints from reputable labs (OpenAI, DeepMind, NVIDIA) with publication date clearly stated
- **FR-021**: All sources MUST be labeled as "established" or "emerging/experimental" based on publication status
- **FR-022**: Tool-specific content (ROS 2, Isaac, Nav2) MUST reference official documentation as primary source
- **FR-023**: All citations MUST include DOI or URL where available
- **FR-024**: Book MUST include a bibliography or references section per chapter

#### Technical Structure
- **FR-025**: Book MUST be structured as Docusaurus-compatible Markdown files with proper frontmatter
- **FR-026**: Book MUST include diagrams for architectures, data flows, and system designs
- **FR-027**: Each chapter MUST begin with clear learning objectives (both conceptual and practical skills)
- **FR-028**: Each chapter MUST end with summary, key takeaways, and review questions/exercises
- **FR-029**: Technical terminology MUST be consistent throughout all chapters
- **FR-030**: Book MUST be deployable to GitHub Pages
- **FR-031**: Book MUST build successfully in Docusaurus without errors or warnings
- **FR-032**: All images and assets MUST be organized in `/static/img/` directory
- **FR-033**: Cross-references between chapters MUST use proper Docusaurus link syntax
- **FR-034**: Book structure MUST support offline reading
- **FR-035**: Each chapter MUST have clear prerequisites listed (conceptual knowledge and technical setup)

### Key Entities

- **Chapter**: Represents a major topic area with learning objectives, content sections, code examples, diagrams, and references. Each chapter is independently readable with stated prerequisites.
- **Code Example**: Tested, language-specified code snippets demonstrating concepts (ROS 2 nodes, URDF files, Python control scripts, etc.)
- **Diagram**: Visual representation of architectures (ROS 2 node graphs, perception pipelines, VLA workflows, robot kinematic chains)
- **Citation**: Reference to authoritative source (academic paper, documentation, industry publication) with DOI/URL
- **Learning Objective**: Measurable outcome statement at chapter start (e.g., "Explain ROS 2 publish-subscribe model")
- **Capstone Project**: Integrative final project demonstrating autonomous humanoid agent with voice command interface, perception, planning, and execution

## Success Criteria *(mandatory)*

### Measurable Outcomes

#### Reader Learning Outcomes (Theory + Practice Balance)
- **SC-001**: Reader completing foundations section can correctly answer at least 80% of quiz questions on Physical AI concepts vs traditional AI (theory)
- **SC-002**: Reader completing ROS 2 chapter can write functional publisher/subscriber nodes in Python and run them successfully in simulation (practice)
- **SC-003**: Reader completing simulation chapter can create, launch, and interact with a basic URDF robot in Gazebo successfully (practice)
- **SC-004**: Reader completing Isaac chapter can describe all three Isaac components and configure a basic Isaac Sim environment (theory + practice)
- **SC-005**: Reader completing VLA chapter can diagram a complete Vision-Language-Action pipeline and explain integration architecture (theory)
- **SC-006**: Reader completing perception chapter can list at least 5 sensors, explain their purposes, and configure object detection in simulation (theory + practice)
- **SC-007**: Reader completing navigation chapter can successfully configure and run Nav2 for waypoint navigation in simulation (practice)
- **SC-008**: Reader completing manipulation chapter can explain inverse kinematics and implement basic pick-and-place in simulation (theory + practice)
- **SC-009**: Reader completing RL chapter can describe domain randomization, configure training environment, and run basic policy training (theory + practice)
- **SC-010**: Reader completing capstone can successfully implement and demonstrate a voice-controlled simulated humanoid completing at least one manipulation task (full integration)

#### Content Quality (Clarified Standards)
- **SC-011**: Book contains zero broken citation links upon initial publication
- **SC-012**: All foundational topics (Physical AI principles, established algorithms) cite peer-reviewed sources; emerging topics clearly dated and labeled
- **SC-013**: Each major robotics chapter (perception, navigation, manipulation) includes optional "Deploying to Hardware" section
- **SC-014**: 50% or more of chapter content by volume includes hands-on implementation guidance (code, configuration, exercises)
- **SC-015**: All code examples execute successfully on specified environment (Ubuntu 22.04, ROS 2 Humble, Gazebo)
- **SC-016**: Each hands-on section includes troubleshooting guidance for at least 3 common errors

#### Technical Deliverables
- **SC-017**: Book builds in Docusaurus without errors or warnings on first deployment attempt
- **SC-018**: All diagrams have descriptive alt text for accessibility
- **SC-019**: Each chapter's learning objectives explicitly state both conceptual and practical skills to be gained
- **SC-020**: Book passes plagiarism checks with 100% original content (excluding citations)

#### Validation and Feedback
- **SC-021**: Technical reviewers rate accuracy as 4.5/5 or higher
- **SC-022**: Technical reviewers confirm 50/50 theory-practice balance across chapters
- **SC-023**: 90% of readers report improved understanding of Physical AI after completing the book (post-publication survey)
- **SC-024**: 80% of readers who attempt hands-on exercises successfully complete them (post-publication tracking)

#### Deployment
- **SC-025**: Book successfully deploys to GitHub Pages and is accessible via standard web browsers
- **SC-026**: Mobile rendering of book content is readable without horizontal scrolling
- **SC-027**: Search functionality in Docusaurus returns relevant results for key technical terms

## Out of Scope *(optional)*

The following are explicitly **not** covered in this book:

- Step-by-step hardware assembly instructions for building physical robots
- Vendor comparison or pricing guides for robot hardware
- Low-level firmware development or motor driver programming
- In-depth ethical or policy analysis of humanoid robotics (brief discussion included, but not primary focus)
- Cloud infrastructure optimization or cost management for robotics workloads
- Product selection guides for specific brands or models
- Legal and regulatory compliance details by jurisdiction
- Manufacturing and supply chain considerations
- Business strategy or go-to-market planning for robotics companies
- Detailed mechanical engineering or materials science for robot construction

## Assumptions *(optional)*

### Reader Prerequisites
- Readers have basic programming knowledge (Python preferred) and can write, debug, and run Python scripts
- Readers have fundamental AI/ML background (neural networks, supervised learning, basic RL concepts)
- Readers have basic mathematics background: comfortable with matrix operations, gradient concepts, probability distributions (no proofs required)
- Readers are familiar with Linux command-line basics or willing to learn via supplementary resources

### Technical Environment
- Readers have access to a computer capable of running simulation software (minimum 8GB RAM, 16GB recommended; dedicated GPU recommended but not required for basic simulations)
- Readers have internet access for downloading tools (ROS 2, simulators, Isaac platform)
- Readers are using Ubuntu 22.04 or compatible Linux distribution for ROS 2 examples (alternatives noted where applicable)
- Readers will install and run simulation tools (Gazebo, optionally Isaac Sim) as part of learning experience

### Content Philosophy (CLARIFIED)
- **Balanced theory and hands-on**: Book provides 50% conceptual understanding / 50% implementation practice
- Readers expected to run code examples and complete hands-on exercises in simulation
- Code examples are tested and executable, not just illustrative
- **Simulation-first with hardware bridging**: Primary development in simulation; optional "deploying to hardware" sections for readers with robot access
- Physical hardware is optional but discussed; book includes practical considerations for real-world deployment

### Citation Standards (CLARIFIED)
- **Tiered source authority approach**:
  - **Foundational topics** (Physical AI principles, robotics fundamentals, established algorithms): Peer-reviewed papers required
  - **Emerging topics** (VLA systems, latest Isaac features, recent LLM integrations): Preprints from reputable labs (OpenAI, DeepMind, NVIDIA) acceptable with clear dating
  - **Tool documentation** (ROS 2, Isaac, Nav2): Official documentation is authoritative
  - All claims clearly labeled as "established" vs "emerging/experimental"
- Readers can access academic papers via institutional subscriptions or open-access sources
- Book content is current as of publication date (2025); readers understand rapid evolution of Physical AI field

## Dependencies *(optional)*

### External Systems and Tools

- **ROS 2 (Humble or newer)**: Required for understanding and running middleware examples
- **Gazebo Classic or Gazebo Sim**: Required for simulation exercises
- **NVIDIA Isaac Sim**: Required for Isaac-specific chapters (optional alternative: use conceptual understanding without hands-on)
- **Python 3.8+**: Required for code examples
- **Docusaurus 2.x**: Required for building and deploying the book
- **Git and GitHub**: Required for version control and deployment

### Content Dependencies

- Academic research papers on Physical AI, embodied intelligence, humanoid robotics
- Official documentation: ROS 2, NVIDIA Isaac, Nav2, OpenCV, PyTorch
- Industry publications from Boston Dynamics, Tesla Optimus, Figure AI, Agility Robotics
- Open-source robotics projects and repositories for reference implementations

### Infrastructure Dependencies

- GitHub repository for hosting book source and deployed site
- GitHub Pages for hosting the published book
- CI/CD pipeline for automated builds (GitHub Actions recommended)

## Notes *(optional)*

### Content Strategy (UPDATED with Clarifications)

- **Chapter Structure**: learning objectives (theory + practice) → conceptual foundations → system architecture → hands-on implementation → optional hardware deployment → review questions/exercises → key takeaways
- **Theory-Practice Balance (50/50)**:
  - Approximately half of chapter content covers concepts, principles, and system architecture
  - Approximately half covers code examples, configuration, hands-on exercises, and troubleshooting
  - Each major topic demonstrates both "why it works" and "how to implement it"
- **Progressive Complexity**: Start with foundational theory, move to system architecture, then to hands-on implementation, culminating in integrated capstone
- **Simulation-First Pedagogy**:
  - Primary teaching happens in simulation (Gazebo/Isaac Sim)
  - Each major robotics chapter includes optional "Deploying to Hardware" section
  - Sim-to-real transfer gets dedicated coverage
  - Physical hardware considerations discussed but not required
- **Connections Emphasis**: Explicitly show how perception feeds navigation, VLA uses perception, etc.

### Technical Considerations (UPDATED)

- **Code Examples**: Prioritize clarity and learnability over performance optimization; all examples tested and executable
- **Troubleshooting**: Each hands-on section includes common errors, debugging tips, and expected outputs
- **Environment Specification**: ROS 2 Humble on Ubuntu 22.04 as baseline; compatibility notes for other versions
- **Diagrams**: Generated using version-control-friendly tools (diagrams-as-code, draw.io with XML export)
- **Citation Format**: Markdown links with visible author-year format: `[Smith et al., 2024](https://doi.org/...)`
- **Source Labeling**: All claims clearly marked as "established" (peer-reviewed) or "emerging" (preprints, recent tools)
- **Version Tracking**: API-specific details noted with version and date (e.g., "ROS 2 Humble as of 2024")

### Risk Mitigation (UPDATED)

- **Risk**: Rapid field evolution makes content outdated
  - **Mitigation**: Focus on foundational concepts over specific API versions; clearly label emerging vs established content; date all version-specific claims; establish content review schedule

- **Risk**: Readers lack prerequisite knowledge
  - **Mitigation**: Explicit prerequisites per chapter (conceptual + technical setup); supplementary resource links; math background guidance

- **Risk**: Theory-practice balance difficult to maintain
  - **Mitigation**: Measure chapter content volume (50/50 target); peer review for balance; success criteria SC-022 validates balance

- **Risk**: Hands-on exercises fail due to environment differences
  - **Mitigation**: Specify exact environment (Ubuntu 22.04, ROS 2 Humble); include troubleshooting for common errors; provide expected outputs

- **Risk**: Simulation-only approach limits real-world applicability
  - **Mitigation**: Include optional "Deploying to Hardware" sections; dedicated sim-to-real coverage; discuss reality gap and physical constraints

- **Risk**: Citation link rot
  - **Mitigation**: Use DOIs where available; archive important links; periodic link checking; label source stability

### Success Metrics Post-Publication

- Reader survey feedback on clarity and usefulness
- GitHub repository stars/forks as engagement indicator
- Community contributions (issues, PRs) as measure of active use
- Citation by other educational resources or papers
- Completion rate tracking if analytics integrated
