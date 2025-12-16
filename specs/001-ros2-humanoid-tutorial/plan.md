
# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-09 | **Spec**: specs/001-ros2-humanoid-tutorial/spec.md
**Input**: Feature specification from `/specs/001-ros2-humanoid-tutorial/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Development of a comprehensive Physical AI & Humanoid Robotics Book using Docusaurus with 4 modules (ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action) and an integrated RAG chatbot. The system will include a FastAPI backend with Qdrant Cloud vector database and Neon Postgres for metadata, providing AI-powered answers based on book content with source highlighting.

## Technical Context

**Language/Version**: Python 3.11 (FastAPI backend), JavaScript/TypeScript (Docusaurus frontend)
**Primary Dependencies**: Docusaurus v3, FastAPI, Qdrant Cloud, Neon Serverless Postgres, OpenAI API, React
**Storage**: Qdrant Cloud (vector embeddings), Neon Serverless Postgres (metadata), GitHub Pages (static content)
**Testing**: pytest (backend), Jest/Cypress (frontend), Documentation verification
**Target Platform**: Web-based (Docusaurus on GitHub Pages, FastAPI backend on cloud platform)
**Project Type**: Web application (frontend Docusaurus + backend FastAPI)
**Performance Goals**: <3s chat response time, <2s page load time, 99% uptime
**Constraints**: <512MB memory for backend, <100MB for static assets, OpenAI token usage limits
**Scale/Scope**: Support 1000+ concurrent users, 100+ book chapters, 5000+ embedding vectors

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Library-First**: ✅ The feature is split into two libraries: frontend (Docusaurus) and backend (FastAPI) that can function independently
- **CLI Interface**: ✅ Backend provides API endpoints (text I/O via JSON), with potential CLI for content ingestion and management
- **Test-First (NON-NEGOTIABLE)**: ✅ All components will follow TDD with tests written before implementation
- **Integration Testing**: ✅ Plan includes integration tests for API contracts, chatbot functionality, and frontend-backend communication
- **Observability**: ✅ Plan includes structured logging, monitoring, and debugging capabilities
- **Simplicity**: ✅ Plan starts with MVP approach, implementing core functionality first before advanced features

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (frontend Docusaurus + backend FastAPI)
backend/
├── src/
│   ├── models/
│   │   ├── chat.py          # Chat session models
│   │   ├── content.py       # Book content models
│   │   └── user.py          # User interaction models
│   ├── services/
│   │   ├── chat_service.py  # Chatbot logic
│   │   ├── embedding_service.py # Vector embedding operations
│   │   ├── database_service.py # Neon Postgres operations
│   │   └── vector_service.py # Qdrant operations
│   ├── api/
│   │   ├── routes/
│   │   │   ├── chat.py      # Chat endpoints
│   │   │   ├── content.py   # Content endpoints
│   │   │   └── health.py    # Health check endpoints
│   │   └── main.py          # FastAPI app entry point
│   └── cli/
│       └── ingestion.py     # Content ingestion CLI
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/  # Docusaurus structure
├── docs/
│   ├── chapter-1-introduction/
│   ├── chapter-2-ros2-basics/
│   ├── chapter-3-python-agents-rclpy/
│   ├── chapter-4-urdf-humanoids/
│   ├── chapter-5-practical-system/
│   └── chapter-6-summary/
├── src/
│   ├── components/
│   │   ├── ChatbotWidget/   # Floating chat widget
│   │   ├── ChatbotPage/     # Full page chat interface
│   │   └── TextSelector/    # Text selection functionality
│   ├── pages/
│   │   └── chatbot.js       # Standalone chatbot page
│   └── css/
├── static/
└── docusaurus.config.js

# Module structure for 4 modules with 3-4 chapters each
docs/
├── module-1-ros2/
│   ├── chapter-1-introduction/
│   ├── chapter-2-nodes-topics/
│   └── chapter-3-services-actions/
├── module-2-digital-twin/
│   ├── chapter-1-gazebo-simulation/
│   ├── chapter-2-unity-integration/
│   └── chapter-3-visualization/
├── module-3-ai-brain/
│   ├── chapter-1-nvidia-isaac/
│   ├── chapter-2-control-systems/
│   └── chapter-3-planning-navigation/
└── module-4-vla/
    ├── chapter-1-vision-language/
    ├── chapter-2-action-planning/
    └── chapter-3-integration/
```

**Structure Decision**: Web application structure selected with separate backend (FastAPI) and frontend (Docusaurus) to allow independent scaling and development. The Docusaurus site will serve static content while the FastAPI backend handles dynamic chatbot functionality and content management.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multi-repo structure | Backend and frontend need different deployment strategies and scaling | Single repo would complicate deployment and team collaboration |
| Multiple database types | Need vector storage (Qdrant) for embeddings and relational storage (Neon) for metadata | Single database would not efficiently handle both vector and relational queries |
