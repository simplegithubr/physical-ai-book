---
id: 1
title: Integrated RAG Chatbot Plan
stage: plan
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: 1-rag-chatbot-book-queries
branch: 1-rag-ingestion-pipeline
user: Claude Code User
command: /sp.plan
labels: [rag, chatbot, planning, api]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-rag-chatbot-book-queries/plan.md
 - specs/1-rag-chatbot-book-queries/research.md
 - specs/1-rag-chatbot-book-queries/data-model.md
 - specs/1-rag-chatbot-book-queries/quickstart.md
 - specs/1-rag-chatbot-book-queries/contracts/openapi.yaml
 - .specify/memory/claude-agent-context.md
tests:
 - null
---

## Prompt

```text
Integrated RAG Chatbot Development for Book Content Queries
Overall Plan
Develop and integrate a high-quality Retrieval-Augmented Generation (RAG) chatbot into the existing Physical AI book site, completing Phase 2 of the project. The chatbot will enable users to query the book's content (full or selected text) via an embeddable widget. Build backend-first for stability, then frontend. Strictly use Cohere APIs (not OpenAI), free-tier services, and adhere to sp.constitution and sp.specify. Ensure seamless embedding into Docusaurus (e.g., via iframe in sidebar or pages).
Phase 1: Project Setup & Secure Configuration (Days 1-2)

Fork/clone book repo or create new repo for chatbot (branch: 1-rag-chatbot-book-queries). Integrate with existing Docusaurus if possible.
Structure:
backend/          # FastAPI API
frontend/         # Static widget (built later)
ingestion/        # Scripts to scrape/ingest book content
tests/            # Test suite
docs/             # Integration guide for Docusaurus
Secure .env (gitignored):
COHERE_API_KEY="bZbc7rCqD1pKlKMwnxZWGITGdTqlFOt68ZQlM1Sv"
QDRANT_URL="https://2871e2b5-ca00-4d40-946c-631332b06e6a.europe-west3-0.gcp.cloud.qdrant.io:6333"
QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.ajA529WzE6x3nN4t0WAk0MWPRjCVROzHG2dGwCMrvS0"
QDRANT_CLUSTER_ID="2871e2b5-ca00-4d40-946c-631332b06e6a"
NEON_DATABASE_URL="postgresql://neondb_owner:npg_KhdgqD4v6sAB@ep-noisy-glade-a4rx4ua8-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"
Dependencies: fastapi, uvicorn, cohere, qdrant-client, python-dotenv, requests/beautifulsoup4 (for scraping if needed), pdfplumber (fallback if book exported to PDF).

Phase 2: Book Content Ingestion Pipeline (Days 3-8) → FR-001, FR-006

Since book is live Docusaurus site, build scraper to extract text:
Crawl starting from root URL, follow /docs links (e.g., introduction, chapters).
Use BeautifulSoup to parse HTML, extract markdown-like text from  or main content.
Handle sections: title, headings, paragraphs, code blocks (preserve formatting).
Metadata: URL/path, chapter name, section hierarchy.

Chunking: 600-800 tokens, 200 overlap, respect section boundaries.
Embed with Cohere embed-english-v3.0.
Qdrant: collection physical_ai_book, points with payload {chunk_text, chunk_id, url, section}.
Neon Postgres: table for book metadata (version, last_ingested, total_pages).
Run ingestion script to index the entire site content. Handle edge cases: dynamic content, images (skip, focus text).

Phase 3: FastAPI Backend – Priority Build (Days 9-18) → FR-002, FR-003, FR-004, FR-007, FR-008

Core endpoints (build and test locally first):
POST /chat
Input: { "query": str, "selected_text": str (optional), "history": list (optional), "book_id": "physical-ai" (fixed) }
Logic:
If selected_text: Embed and use solely as context.
Else: Embed query, retrieve top-8 chunks from Qdrant.
Prompt: "Answer strictly from context. Cite sources with section/URL. If unknown, say so."
Generate with Cohere command-r-plus.
Output: { "response": str, "sources": [{section, excerpt, url}] }
POST /ingest: Trigger re-indexing of book site.
GET /health.

CORS for Vercel domain. Async ops, retries, logging.
Optimize for <5s responses.

Phase 4: Custom Embeddable Frontend Widget (Days 19-26) → User Story 3, FR-005

Build after backend stable.
Static HTML/JS: Tailwind/hmtx/Alpine.js via CDN.
Features: Chat interface, selected_text via postMessage (for book selection events).
Embed code: iframe in Docusaurus (e.g., add to sidebar.mdx or custom component).
Test integration on local Docusaurus clone.



Phase 6: Testing & Validation (Days 31-40)

50+ tests for User Stories/Edge Cases.
Verify all SC: accuracy ≥95%, no hallucination, <5s, formats, security.
Usability demos on live site.

Final Deliverables

Integrated chatbot in updated book site.
Repo with code, docs, test report.
All requirements met.

Strictly Not Building

Auth, mobile apps, paid tiers, OpenAI.
```

## Response snapshot

Created comprehensive implementation plan for the RAG chatbot system including:
- plan.md with technical context and constitution check
- research.md with technology research and decisions
- data-model.md with entity definitions
- API contracts in OpenAPI YAML format
- quickstart.md for quick setup instructions
- Updated agent context with new technologies

## Outcome

- ✅ Impact: Complete planning phase with all required artifacts created
- 🧪 Tests: N/A for planning phase
- 📁 Files: 6 files created/updated across documentation and configuration
- 🔁 Next prompts: /sp.tasks to generate implementation tasks
- 🧠 Reflection: Planning phase completed successfully with all constitutional requirements verified

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All constitutional checks passed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin task generation with /sp.tasks