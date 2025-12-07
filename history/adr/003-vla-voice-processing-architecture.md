# ADR-003: VLA Voice Processing Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** vla-integration
- **Context:** Need to establish an architecture for processing voice commands through the complete VLA pipeline. The architecture must handle audio input, convert to text, process with LLM, generate action sequences, execute via ROS 2, and provide feedback - all in a simulation environment suitable for educational purposes.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Architecture Pattern: Voice Input → Whisper API → Natural Language Processing → Claude LLM → Action Sequence → ROS 2 Actions → Simulated Robot → Feedback
- Service Structure: Separate services for voice processing, cognitive planning, action execution, and feedback
- API Design: REST-based services following the contracts defined in specs/001-vla-integration/contracts/
- State Management: Entity tracking through VoiceCommand, ActionSequence, ROS2Action, CognitivePlan, and SimulationFeedback as defined in data model

## Consequences

### Positive

- Modularity: Each component can be developed, tested, and modified independently
- Educational Value: Clear separation of concerns helps students understand each component
- Extensibility: New components can be added without affecting the overall architecture
- Testability: Each service can be tested independently with mock dependencies
- Simulation Integration: Architecture works well with ROS 2 and simulation environments

### Negative

- Complexity: Multiple service interactions increase system complexity
- Latency: Multiple service calls may increase response time
- Coordination: Need for careful state management across services
- Debugging: Distributed architecture makes debugging more complex

## Alternatives Considered

- Alternative A: Monolithic architecture with all components in single service
  - Why rejected: Less educational value, harder to understand individual components
- Alternative B: Event-driven architecture with message queues
  - Why rejected: Higher complexity, overkill for educational context
- Alternative C: Direct integration without service boundaries
  - Why rejected: Reduced modularity and educational clarity

## References

- Feature Spec: specs/001-vla-integration/spec.md
- Implementation Plan: specs/001-vla-integration/plan.md
- Related ADRs: ADR-001 (VLA Technology Stack), ADR-002 (Cognitive Planning Approach)
- Evaluator Evidence: specs/001-vla-integration/research.md, specs/001-vla-integration/data-model.md