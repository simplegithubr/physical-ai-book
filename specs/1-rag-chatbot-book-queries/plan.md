# Implementation Plan: Integrated RAG Chatbot Development for Book Content Queries

**Branch**: `1-rag-chatbot-book-queries` | **Date**: 2025-12-19 | **Spec**: [link to spec.md]

**Input**: Feature specification from `/specs/1-rag-chatbot-book-queries/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Develop and integrate a high-quality Retrieval-Augmented Generation (RAG) chatbot into the existing Physical AI book site. The chatbot will enable users to query the book's content (full or selected text) via an embeddable widget. Build backend-first for stability, then frontend, using Cohere APIs exclusively and free-tier services. Ensure seamless embedding into Docusaurus via iframe.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/HTML/CSS
**Primary Dependencies**: FastAPI, Cohere, Qdrant-client, python-dotenv, BeautifulSoup4, pdfplumber, uvicorn
**Storage**: Qdrant Vector Database, Neon Serverless Postgres
**Testing**: pytest
**Target Platform**: Linux server (backend), Cross-browser compatible (frontend widget)
**Project Type**: Web application (backend + frontend widget)
**Performance Goals**: Response time under 5 seconds, handle up to 100 concurrent users on free tiers
**Constraints**: <5s p95 response time, Cohere API rate limits, free tier usage limits, no OpenAI APIs
**Scale/Scope**: Up to 100 concurrent users, full Physical AI book content indexing

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Post-Design Evaluation:

1. **Content Accuracy and Context-Awareness**: ✅ PASSED
   - Design includes BookChunk entity with proper content storage and source tracking
   - ChatResponse includes sources with section, excerpt, and URL for citation
   - API contract enforces response generation from book content only

2. **Resource Efficiency**: ✅ PASSED
   - Design uses Cohere embed-english-v3.0 and command-r-plus within free tier
   - Includes caching strategies in architecture decisions
   - API contract includes rate limiting headers

3. **User-Centric Design**: ✅ PASSED
   - Frontend widget designed for intuitive chat interface
   - Support for selected_text via postMessage for book selection events
   - Embeddable via iframe in Docusaurus

4. **Security and Privacy**: ✅ PASSED
   - No persistent user data storage in data model
   - API key security handled via environment variables
   - Secure API authentication via API key header

5. **Technology Stack Standards**: ✅ PASSED
   - Cohere APIs exclusively used (embed-english-v3.0, command-r-plus)
   - Code modularity achieved through separate models/services/api layers
   - Testing rigor enforced with pytest and contract testing

6. **Development Workflow**: ✅ PASSED
   - Performance targets under 5 seconds included in data model
   - Documentation standards maintained with inline comments and README
   - End-to-end testing covered in API contracts and data model

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-chatbot-book-queries/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── main.py
├── tests/
│   ├── unit/
│   └── integration/
└── requirements.txt

frontend/
├── static/
│   ├── js/
│   ├── css/
│   └── index.html
└── assets/

ingestion/
├── scripts/
│   └── book_scraper.py
├── chunker.py
└── embedder.py

tests/
├── contract/
├── integration/
└── unit/

docs/
└── integration-guide.md
```

**Structure Decision**: Web application structure with separate backend (FastAPI) and frontend (static widget) components, plus ingestion pipeline for book content processing.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |