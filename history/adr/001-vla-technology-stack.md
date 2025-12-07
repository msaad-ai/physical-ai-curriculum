# ADR-001: VLA Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** vla-integration
- **Context:** Need to establish a technology stack for the Vision-Language-Action (VLA) module that integrates Large Language Models with humanoid robots for voice commands and cognitive planning. The stack must support educational content creation, simulation-based validation, and be consistent with existing modules while enabling voice processing, LLM integration, and ROS 2 action execution.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Language: Python 3.8+ for ROS 2 integration and Markdown for documentation
- Voice Processing: OpenAI Whisper API for voice-to-text conversion
- LLM: Anthropic Claude API for natural language to action sequence translation
- Robotics Framework: ROS 2 Humble for action execution
- Simulation: Gazebo Classic/Ignition for testing and validation
- Target Platform: Linux/Ubuntu for ROS 2, Docusaurus for documentation deployment

## Consequences

### Positive

- Consistency with existing modules and ROS 2 ecosystem
- API-based services provide reliability and maintenance
- Claude API aligns with educational context where students use Claude Code
- Gazebo integration with ROS 2 provides validated simulation environment
- Python ecosystem has strong support for AI/ML integration

### Negative

- API dependencies create external service requirements and potential costs
- Version compatibility constraints between ROS 2 Humble and other components
- Platform dependency on Linux/Ubuntu for full functionality
- Potential vendor lock-in to OpenAI and Anthropic APIs

## Alternatives Considered

- Alternative Stack A: Local Whisper model + Local LLM (Llama) + Isaac Sim
  - Why rejected: More complex setup, higher computational requirements, less suitable for educational context
- Alternative Stack B: OpenAI GPT + Custom voice processing + Gazebo only
  - Why rejected: Less consistency with educational tools, custom voice processing adds complexity
- Alternative Stack C: Rule-based action mapping + Speech recognition libraries + Multiple simulators
  - Why rejected: Less flexibility, reduced educational value of LLM integration

## References

- Feature Spec: specs/001-vla-integration/spec.md
- Implementation Plan: specs/001-vla-integration/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/001-vla-integration/research.md