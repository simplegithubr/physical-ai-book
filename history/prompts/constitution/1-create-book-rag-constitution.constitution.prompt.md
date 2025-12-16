---
id: 1
title: Create Book RAG Constitution
stage: constitution
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: User
command: /sp.constitution
labels: ["constitution","book","rag","ai","robotics"]
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
Project: Book + Embedded RAG Chatbot for "Physical AI & Humanoid Robotics"

Core principles:
- Technical accuracy (ROS 2, Gazebo, Unity, Isaac, VLA, LLMs)
- Clarity for CS/AI students
- Reproducible code and setups
- Source verification using official docs

Key standards:
- Book built with Docusaurus, deployed on GitHub Pages
- Written using Spec-Kit Plus + Claude Code
- RAG chatbot uses OpenAI Agents/ChatKit, FastAPI, NeonDB, Qdrant
- Chatbot answers only from book content or user-selected text
- Tested code snippets (ROS 2, Python, Isaac, Nav2, Whisper)

Constraints:
- Must cover 4 modules:
  1. ROS 2 Nervous System
  2. Digital Twin (Gazebo/Unity)
  3. NVIDIA Isaac AI-Robot Brain
  4. Vision-Language-Action
- Include tutorials, diagrams, workflows, simulations
- Full deployment with embedded chatbot

Success criteria:
- Book live + fully navigable
- RAG chatbot accurate, no hallucinations
- All examples reproducible
- End-to-end humanoid workflow: voice → plan → ROS 2 actions → simulation
```

## Response snapshot

Updated the project constitution with principles and standards for the Book + Embedded RAG Chatbot project. The new constitution includes:

- Core Principles: Technical Accuracy, Student Clarity, Reproducible Code, Source Verification, Complete Integration, and No Hallucination Guarantee
- Technology Stack Standards: Docusaurus, GitHub Pages, Spec-Kit Plus + Claude Code, OpenAI Agents/ChatKit, FastAPI, NeonDB, Qdrant
- Development Workflow: Testing requirements and module coverage
- Governance: Compliance and review expectations

The constitution was successfully written to .specify/memory/constitution.md with version 1.0.0, ratified on 2025-12-12.

## Outcome

- ✅ Impact: New project constitution created with 6 core principles for AI & robotics book project
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Can proceed with feature specification using /sp.specify
- 🧠 Reflection: Constitution aligns with project requirements for technical accuracy, student clarity, reproducibility, and source verification

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed with feature specification