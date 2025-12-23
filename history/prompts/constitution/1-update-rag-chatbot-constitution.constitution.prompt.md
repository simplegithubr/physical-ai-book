---
id: 1
title: Update RAG Chatbot Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: 1-rag-ingestion-pipeline
user: Claude Code
command: sp.constitution
labels: ["constitution", "rag", "chatbot", "principles"]
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

Project: Integrated RAG Chatbot Development for Book Content Queries
Core principles:

Functionality focused on accurate, context-aware responses using book's content only when specified.
Efficiency in resource usage (leverage free tiers without exceeding limits).
User-centric design (intuitive interface for queries and text selection).
Security and privacy (no storage of user data beyond sessions; secure API keys).

Key standards:

All responses must be generated using Cohere APIs exclusively (no OpenAI fallback).
Code modularity: Separate concerns (e.g., embedding pipeline, API endpoints, frontend UI).
Testing rigor: Unit tests for retrieval accuracy (e.g., via cosine similarity thresholds in Qdrant) and integration tests for end-to-end flow.
Documentation: Inline comments and README with setup instructions, including Cloud IDE deployment.
Performance: Response time under 5 seconds; handle up to 100 concurrent users on free tiers.

Constraints:

Tech stack: Cohere API, FastAPI, Neon Serverless Postgres, Qdr

## Response snapshot

Updated the project constitution to reflect the RAG Chatbot development for book content queries. The new constitution includes principles for content accuracy, resource efficiency, user-centric design, and security. It also defines the technology stack standards, development workflow, and governance requirements for the project.

## Outcome

- ✅ Impact: Updated project constitution with new RAG-focused principles and governance
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new constitution
- 🔁 Next prompts: Can proceed with spec, plan, and task creation for the RAG chatbot
- 🧠 Reflection: Constitution now properly reflects the RAG chatbot project instead of the previous ROS/Isaac robotics focus

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification based on updated constitution