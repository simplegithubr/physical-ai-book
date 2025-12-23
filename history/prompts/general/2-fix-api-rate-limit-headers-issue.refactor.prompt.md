---
id: 2
title: Fix API Rate Limit Headers Issue
stage: refactor
date: 2025-12-21
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-rag-ingestion-pipeline
user: SG
command: manual
labels: ["api", "rate-limiting", "headers", "refactor"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/src/api/chat.py
 - backend/src/api/ingest.py
 - backend/src/api/widget.py
tests:
 - none
---

## Prompt

Fixed backend API rate limit headers issue where Pydantic response models were being passed to add_rate_limit_headers function which expected response objects with headers attribute.

## Response snapshot

Updated all API endpoints that return Pydantic models or dictionaries to convert them to JSONResponse objects before adding rate limit headers. This fixes the "'ChatResponse' object has no attribute 'headers'" error.

## Outcome

- ✅ Impact: Fixed API rate limiting functionality by properly handling response objects that don't have headers attribute
- 🧪 Tests: none
- 📁 Files: Modified chat.py, ingest.py, and widget.py to properly handle rate limit headers
- 🔁 Next prompts: none
- 🧠 Reflection: When using middleware that adds headers, ensure response objects support headers attribute

## Evaluation notes (flywheel)

- Failure modes observed: Pydantic models and dict responses don't have headers attribute causing runtime errors
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add type checking to rate limit function to handle different response types