# Data Model: Module 2 Digital Twin (Gazebo & Unity)

**Created**: 2025-12-06
**Feature**: Module 2: Digital Twin (Gazebo & Unity)
**Status**: Complete

## Entity: Chapter Content

**Description**: Educational content for each chapter in the digital twin module

**Fields**:
- `id` (string): Unique identifier for the chapter (e.g., "module2-intro", "module2-gazebo-setup")
- `title` (string): Display title of the chapter
- `content` (string): Main educational content in Markdown format
- `diagrams` (array): List of diagram references used in the chapter
- `code_examples` (array): List of code example objects
- `exercises` (array): List of exercise objects
- `glossary` (object): Key terms and definitions for the chapter
- `summary` (string): Chapter summary
- `section_structure` (object): Contains the 7 required sections

**Validation Rules**:
- Must include all 7 required sections: Concept Introduction, Importance for Physical AI, Implementation Breakdown, Real-World Use Cases, Student Exercises, Glossary, Summary
- Content must be in valid Markdown format
- All referenced diagrams must exist
- All code examples must be valid and executable

**Relationships**:
- Contains multiple Code Example entities
- Contains multiple Exercise entities
- References multiple Diagram entities

## Entity: Code Example

**Description**: Code snippets and examples used throughout the module

**Fields**:
- `id` (string): Unique identifier for the example
- `language` (string): Programming/scripting language (e.g., "python", "xml", "bash", "csharp")
- `platform` (string): Target platform ("gazebo", "unity", "ros2", or "common")
- `purpose` (string): Educational objective of the example
- `code` (string): The actual code content
- `explanation` (string): Explanation of the code functionality
- `complexity_level` (string): Difficulty level ("beginner", "intermediate", "advanced")
- `validation_steps` (array): Steps to verify the example works correctly

**Validation Rules**:
- Code must be syntactically correct for the specified language
- Must include explanation of what the code does
- Must be executable in the target environment
- Complexity level must match educational objectives

**Relationships**:
- Belongs to a Chapter Content entity

## Entity: Exercise

**Description**: Student exercises and mini-simulation tasks

**Fields**:
- `id` (string): Unique identifier for the exercise
- `title` (string): Exercise title
- `difficulty` (string): Difficulty level ("beginner", "intermediate", "advanced")
- `objectives` (array): Learning objectives the exercise addresses
- `instructions` (string): Step-by-step instructions for the student
- `expected_outcomes` (array): What the student should achieve
- `validation_steps` (array): How to verify the exercise was completed correctly
- `prerequisites` (array): What knowledge/skills are needed
- `estimated_duration` (number): Time needed to complete in minutes

**Validation Rules**:
- Must have clear, achievable objectives
- Instructions must be detailed enough for beginners
- Expected outcomes must be measurable
- Exercise must be completable with provided resources

**Relationships**:
- Belongs to a Chapter Content entity

## Entity: Diagram

**Description**: Visual aids and architectural diagrams for the module

**Fields**:
- `id` (string): Unique identifier for the diagram
- `title` (string): Diagram title
- `type` (string): Diagram type ("architecture", "flowchart", "sequence", "conceptual")
- `description` (string): Explanation of what the diagram illustrates
- `content` (string): Diagram content (Mermaid, ASCII, or image reference)
- `caption` (string): Caption to appear under the diagram
- `use_case` (string): Which concept or section the diagram supports

**Validation Rules**:
- Content must be in valid diagram format (Mermaid, ASCII art, or proper image reference)
- Must clearly illustrate the intended concept
- Must have appropriate caption explaining the diagram

**Relationships**:
- Referenced by multiple Chapter Content entities

## Entity: Sensor Simulation

**Description**: Virtual sensors used in the digital twin simulation

**Fields**:
- `id` (string): Unique identifier for the sensor
- `type` (string): Sensor type ("lidar", "depth_camera", "imu")
- `parameters` (object): Configuration parameters for the sensor
- `platform` (string): Platform where sensor is implemented ("gazebo", "unity", "both")
- `data_output` (string): Description of data produced by the sensor
- `educational_purpose` (string): Learning objective for this sensor
- `example_usage` (string): How the sensor is used in examples/exercises

**Validation Rules**:
- Parameters must be valid for the sensor type
- Data output must match real-world sensor behavior
- Must align with educational objectives

**Relationships**:
- Used in Code Example entities
- Referenced in Exercise entities

## Entity: Simulation Environment

**Description**: Configuration for Gazebo and Unity simulation environments

**Fields**:
- `id` (string): Unique identifier for the environment
- `platform` (string): "gazebo" or "unity"
- `version_requirement` (string): Required version of the platform
- `robot_model` (string): Robot model used in this environment
- `physics_settings` (object): Physics configuration (medium fidelity as determined)
- `setup_instructions` (string): How to set up the environment
- `validation_steps` (array): How to verify the environment is working

**Validation Rules**:
- Version requirements must be compatible with educational objectives
- Setup instructions must be clear and achievable
- Physics settings must match the medium fidelity requirement

**Relationships**:
- Referenced in Chapter Content entities
- Contains multiple Sensor Simulation entities