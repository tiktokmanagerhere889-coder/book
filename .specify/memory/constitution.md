<!-- SYNC IMPACT REPORT:
Version change: N/A -> 1.0.0
Modified principles: N/A
Added sections: All principles and sections based on AI-Native Interactive Book project
Removed sections: Template placeholders
Templates requiring updates: N/A (initial constitution)
Follow-up TODOs: None
-->
# AI-Native Interactive Book with Embedded RAG Intelligence Constitution

## Core Principles

### I. Spec-Driven Development
All development follows a structured specification-first approach; Features and changes must be documented in specs before implementation; Clear requirements and acceptance criteria are mandatory before development begins.

### II. Accuracy and Content Grounding
All book content must be technically accurate and verifiable; RAG chatbot responses must be strictly grounded in book content; No hallucinations or fabricated information allowed in any AI-generated content.

### III. Reusable AI Intelligence
Subagents for authoring, validation, RAG, and translation must be isolated and reusable; Agent Skills must be documented and maintainable; Intelligence components should be modular and independently deployable.

### IV. Personalization-First Design
User experience must prioritize personalization capabilities; Authentication and user profiles enable tailored content delivery; Personalization features are core functionality, not afterthoughts.

### V. Production-Grade Architecture
All systems must meet production readiness standards; Proper error handling, monitoring, and observability required; Security, performance, and reliability are foundational requirements.

### VI. Modular Book Structure
Content must follow modular, chapter-based organization; Docusaurus framework provides consistent presentation layer; Technical accuracy verification processes are built into the authoring workflow.

## Additional Standards

### Book Authoring and RAG Implementation
- Written with Docusaurus using Spec-Kit Plus methodology
- Authored and refactored by Claude Code with technical accuracy verification
- RAG chatbot stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- Safe failure mechanisms when context is insufficient

### Localization and Personalization
- Support for multilingual content with English as canonical source
- Logged-in users can translate chapters to Urdu with proper localization
- Personalization features available for authenticated users
- User signup collects software and hardware background for personalization

### Technology Stack Constraints
- Deployment: GitHub Pages
- Frontend: Docusaurus
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free Tier)
- Auth: Better-Auth for signup/signin functionality

## Development Workflow

### Implementation Standards
- All features must follow spec-driven development lifecycle (spec → plan → tasks → implementation)
- Reusable agents and skills must be properly isolated and documented
- Personalization and translation features must be functional before release
- Claude Code must be used for all development tasks

### Quality Requirements
- Book must be published and accessible via GitHub Pages
- RAG chatbot must provide accurate answers from book content
- Personalization and Urdu translation must function correctly
- All components must be implemented via Spec-Kit Plus methodology

## Governance

This constitution governs all development activities for the AI-Native Interactive Book project. All implementation must align with these principles and standards. Changes to this constitution require explicit approval and documentation of the rationale. All team members must adhere to spec-driven development practices and the technology stack constraints outlined herein.

**Version**: 1.0.0 | **Ratified**: 2026-01-13 | **Last Amended**: 2026-01-13