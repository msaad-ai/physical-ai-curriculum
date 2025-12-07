# Research: ROS 2 Nervous System Module

## Decision: Focus on ROS 2 fundamentals for Module 1
**Rationale**: To avoid overlap with Module 2 & 3 and maintain clear learning progression. Module 1 establishes foundational knowledge before advancing to simulation and AI topics.
**Alternatives considered**: Including advanced topics in Module 1, combining modules - rejected as it would overwhelm beginners.

## Decision: Use simple code examples instead of full applications
**Rationale**: Simple examples help beginners understand concepts without getting lost in complex implementation details. Examples can be gradually built upon in later modules.
**Alternatives considered**: Full application examples - rejected as they would obscure learning objectives with implementation complexity.

## Decision: Focus on humanoid context rather than general robotics
**Rationale**: Humanoid robotics provides a concrete, relatable context that helps students understand how ROS 2 concepts apply to real-world robots. The "nervous system" metaphor is particularly apt for humanoid robots.
**Alternatives considered**: General robotics examples - rejected as they would be less engaging and harder to visualize for beginners.

## Decision: Cover only nodes, topics, services, and parameters (not Actions)
**Rationale**: Actions are more complex and can be introduced in later modules after students have mastered the fundamentals. This maintains focus on the core communication patterns.
**Alternatives considered**: Including Actions in Module 1 - rejected as it would overload beginners with too many concepts.

## Decision: Use rclpy (Python) for all examples
**Rationale**: Python is more beginner-friendly than C++ and is well-supported in ROS 2. The specification explicitly requires rclpy usage.
**Alternatives considered**: C++ examples - rejected as per specification constraints.

## Decision: Create 4 specific chapters as outlined
**Rationale**: The 4-chapter structure provides logical progression from concepts to implementation to application. Each chapter builds on the previous one while maintaining focus.
**Alternatives considered**: Different chapter organization - rejected as the specified structure aligns with learning objectives.

## Technical Research Findings

### ROS 2 Humble Installation
- Standard installation via apt packages on Ubuntu 22.04
- Python 3.10 required for ROS 2 Humble
- Colcon build system for workspace management

### Node Architecture
- Nodes are processes that perform computation
- Nodes communicate via topics (publish/subscribe), services (request/response), and parameters
- Each node runs in its own process and can be written in different languages

### Topic Communication
- Unidirectional, asynchronous communication
- Publishers send messages to subscribers
- Multiple publishers and subscribers can use the same topic
- Data types defined by message definitions (.msg files)

### Service Communication
- Synchronous request/response pattern
- One service server, one client per service call
- Used for actions that require confirmation or return values

### URDF Basics
- Unified Robot Description Format
- XML-based format describing robot structure
- Includes links (rigid bodies) and joints (constraints)
- Used for simulation, visualization, and kinematics

### Docusaurus Markdown Requirements
- Frontmatter with title and description
- Proper heading hierarchy (h1-h6)
- Code blocks with language specification
- Image embedding with relative paths