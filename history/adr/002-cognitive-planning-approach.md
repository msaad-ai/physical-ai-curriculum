# ADR-002: Cognitive Planning Approach

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** vla-integration
- **Context:** Need to establish an approach for translating natural language commands into executable ROS 2 action sequences. The approach must balance simplicity for educational purposes with effectiveness for cognitive planning in robotics, while leveraging the chosen LLM technology.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Approach: Structured prompt engineering for LLM-to-ROS 2 action mapping
- Implementation: Template-based system where LLM receives natural language command and available ROS 2 actions
- Validation: Action sequence validation before execution to ensure safety and feasibility
- Framework: Cognitive planning module that generates action sequences with confidence scoring

## Consequences

### Positive

- Educational value: Students can understand and modify the prompt templates
- Flexibility: Easy to extend with new action types and capabilities
- Integration: Works well with Claude's reasoning capabilities
- Safety: Validation layer prevents invalid action sequences
- Debugging: Clear traceability from natural language to actions

### Negative

- Complexity: Requires careful prompt engineering and maintenance
- Reliability: Dependent on LLM's interpretation consistency
- Performance: Potential latency in processing complex commands
- Predictability: Less deterministic than rule-based systems

## Alternatives Considered

- Alternative A: Rule-based mapping system with predefined command patterns
  - Why rejected: Less flexible, limited educational value, difficult to handle complex commands
- Alternative B: Machine learning models trained specifically for action mapping
  - Why rejected: Higher complexity, more data requirements, less interpretable for educational context
- Alternative C: Hybrid approach combining rules and LLMs
  - Why rejected: Added complexity without clear benefits over structured prompts

## References

- Feature Spec: specs/001-vla-integration/spec.md
- Implementation Plan: specs/001-vla-integration/plan.md
- Related ADRs: ADR-001 (VLA Technology Stack)
- Evaluator Evidence: specs/001-vla-integration/research.md