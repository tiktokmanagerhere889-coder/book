---
id: 1
title: Update Project Constitution
stage: constitution
date: 2026-01-13
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: hassan
command: sp.constitution
labels: [constitution,spec-driven-development,ai-book,rag]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: AI-Native Interactive Book with Embedded RAG Intelligence

Purpose:
Author, deploy, and operate an AI-native technical book using Spec-Kit Plus and Claude Code, published via Docusaurus on GitHub Pages, with personalized, multilingual content and an embedded RAG chatbot.

Core principles:
- Spec-driven development
- Accuracy and content grounding
- Reusable AI intelligence
- Personalization-first design
- Production-grade architecture

Key standards:
Book Authoring:
- Written with Docusaurus using Spec-Kit Plus
- Authored and refactored by Claude Code
- Modular, chapter-based structure
- Verifiable technical accuracy
- Personalization-ready content

RAG Chatbot:
- Answers strictly from book content
- Supports full-book, chapter, and selected-text queries
- Stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- Safe failure when context is insufficient

Reusable Intelligence:
- Claude Code Subagents for authoring, validation, RAG, and translation
- Agent Skills must be isolated, reusable, and documented

Auth & Personalization:
- Signup/Signin via Better-Auth
- Signup collects software and hardware background
- Logged-in users can personalize each chapter via a button

Localization:
- Logged-in users can translate chapters to Urdu
- English remains the canonical source

Constraints:
- Deployment: GitHub Pages
- Frontend: Docusaurus
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free Tier)

Success criteria:
- Book published and accessible
- RAG chatbot answers accurately
- Personalization and Urdu translation function correctly
- Reusable agents and skills are operational
- All implemented via spec-kit plus
```

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the specific details for the AI-Native Interactive Book project. The constitution now includes six core principles aligned with the project goals: Spec-Driven Development, Accuracy and Content Grounding, Reusable AI Intelligence, Personalization-First Design, Production-Grade Architecture, and Modular Book Structure. Additional sections cover standards for book authoring, RAG implementation, localization, technology stack constraints, and development workflow. The constitution is versioned as 1.0.0 and dated 2026-01-13.

## Outcome

- ✅ Impact: Created initial project constitution with all specified principles and standards
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Proceed with creating spec for the AI-native book project
- 🧠 Reflection: Constitution now properly captures all core principles and constraints from user requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec creation based on this constitution