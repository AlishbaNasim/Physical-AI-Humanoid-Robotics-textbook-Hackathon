<!--
Sync Impact Report:
Version change: 1.0.1 -> 1.1.0
Modified principles: Library-First, CLI Interface → Physical AI & Humanoid Robotics Book Principles
Added sections: I. Accuracy, II. Completeness, III. Clarity, IV. Consistency, V. Reproducibility, VI. Spec-Driven Development, VII. RAG Chatbot Integration
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/sp.constitution.md: ✅ updated
  - CLAUDE.md: ✅ updated
  - README.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Accuracy
All explanations must be based on verified robotics and simulation frameworks including ROS 2, Gazebo, NVIDIA Isaac, Unity Robotics, and Vision-Language-Action systems; All module content must be technically reliable, academically valid, and aligned with real-world humanoid robotics practices

### II. Completeness
Total: 4 Modules, each containing 3–4 chapters; All chapters will later be generated topic-by-topic; Must include diagrams, step-by-step instructions, architectures, workflows, and system explanations

### III. Clarity
Content must be written for advanced AI students who already know machine learning fundamentals but are new to Physical AI, humanoid control, and VLA models; Use clear, accessible language while maintaining technical precision

### IV. Consistency
Use a clean, modern, structured layout; Consistent tone, formatting, diagrams, headings, and terminology across all chapters; Maintain uniform architecture and design patterns throughout the Docusaurus site

### V. Reproducibility
All technical modules must include runnable code samples, commands, simulation steps, and architecture diagrams; Instructions should be replicable by students without external assumptions; All examples must be tested and verified

### VI. Spec-Driven Development
Entire planning, breakdown, task creation, and implementation must follow the Spec-Kit Plus workflow; All outputs must follow /sp.plan → /sp.tasks → /sp.implement → /sp.test → /sp.release

### VII. RAG Chatbot Integration
Chatbot must be embedded directly into the Docusaurus site; Should answer strictly using the book's content—no external knowledge; Must support: user selects text → chatbot answers using only that selected section; Backend uses FastAPI + Neon Postgres + Qdrant Cloud; AI reasoning uses OpenAI Agents / ChatKit

## Additional Constraints

All code must adhere to the best practices for web development, robotics frameworks, and AI/ML implementations. Technical content must be validated against actual ROS 2 implementations, simulation environments, and current Physical AI research.

## Development Workflow

Code reviews are mandatory for all changes. All pull requests require at least one approval before merging. Technical content must be validated by domain experts before publication. All examples and code snippets must be tested in actual environments.

## Governance

This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Use CLAUDE.md for runtime development guidance.

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-09
