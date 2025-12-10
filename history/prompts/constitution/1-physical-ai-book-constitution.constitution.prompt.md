---
id: 1
title: "Physical AI Book Constitution"
stage: constitution
date: "2025-12-09"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-ros2-humanoid-tutorial"
user: "Administrator"
command: "/sp.constitution"
labels: ["constitution", "physical-ai", "humanoid-robotics", "book"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Physical AI & Humanoid Robotics Book Constitution

## User Input
```
Project: Physical AI & Humanoid Robotics Book + Integrated RAG Chatbot

Technology Stack:
- Docusaurus (v3)
- React
- Tailwind CSS (optional)
- FastAPI backend
- Qdrant Cloud (vector DB)
- Neon Serverless Postgres
- OpenAI Agents / ChatKit SDK

Core Principles:

Accuracy:
- All explanations must be based on verified robotics and simulation frameworks including ROS 2, Gazebo, NVIDIA Isaac, Unity Robotics, and Vision-Language-Action systems.
- All module content must be technically reliable, academically valid, and aligned with real-world humanoid robotics practices.

Completeness:
- Total: 4 Modules, each containing 3–4 chapters.
- All chapters will later be generated topic-by-topic.
- Must include diagrams, step-by-step instructions, architectures, workflows, and system explanations.

Clarity:
- Content must be written for advanced AI students who already know machine learning fundamentals but are new to Physical AI, humanoid control, and VLA models.

Consistency:
- Use a clean, modern, structured layout.
- Consistent tone, formatting, diagrams, headings, and terminology across all chapters.

Reproducibility:
- All technical modules must include runnable code samples, commands, simulation steps, and architecture diagrams.
- Instructions should be replicable by students without external assumptions.

Spec-Driven:
- Entire planning, breakdown, task creation, and implementation must follow the Spec-Kit Plus workflow.
- All outputs must follow /sp.plan → /sp.tasks → /sp.implement → /sp.test → /sp.release.

RAG Chatbot Requirements:
- Chatbot must be embedded directly into the Docusaurus site.
- Should answer strictly using the book's content—no external knowledge.
- Must support: user selects text → chatbot answers using only that selected section.
- Backend uses FastAPI + Neon Postgres + Qdrant Cloud.
- AI reasoning uses OpenAI Agents / ChatKit.
```

## Response
Updated the project constitution to reflect the Physical AI & Humanoid Robotics Book project requirements, replacing the generic website constitution with specific principles for accuracy, completeness, clarity, consistency, reproducibility, spec-driven development, and RAG chatbot integration.

## Outcome
Successfully created a project constitution that aligns with the Physical AI & Humanoid Robotics Book requirements, incorporating all specified principles and constraints.

## Evaluation
The constitution update was completed successfully with all core principles properly documented and aligned with the project requirements.