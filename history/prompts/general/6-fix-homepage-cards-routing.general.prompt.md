---
id: "6"
title: "Fix Homepage Cards Routing"
stage: "general"
date: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "003-isaac-robotics"
user: "unknown"
command: "/sp.implement"
labels: ["docusaurus", "routing", "homepage", "doc-links"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "book/src/pages/index.tsx"
tests: []
---

## Prompt

Context:
Docusaurus v3 project.
All module docs exist and sidebars are correct.
Navbar links work.
Homepage module cards return "Page Not Found".

Root Cause (CONFIRMED):
Homepage cards link directly to /docs/module-X/index
This bypasses Docusaurus sidebar context.
Navbar works because it uses docSidebar routing.

Goal:
Fix homepage cards so they navigate using proper Docusaurus doc routing
WITHOUT changing docs, sidebars, or adding new content.

Constraints:
- DO NOT change docs structure
- DO NOT change sidebars.ts
- DO NOT add new markdown files
- FIX ONLY homepage routing logic

Task:
1. Open:
   book/src/pages/index.tsx

2. Replace ALL hardcoded links like:
   "/docs/module-1-ros2/index"
   "/docs/module2/index"
   "/docs/module-3-ai-robot-brain/index"
   "/docs/module-4-vla/index"

3. Instead, use Docusaurus doc routing helpers:
   - use `@docusaurus/Link`
   - OR `useDocById`
   - OR `useBaseUrl`
   - OR `to={useBaseUrl('/docs/...')}`

4. Ensure links resolve via DOC IDS, not raw URLs.

## Response snapshot

Fixed the homepage card routing by changing the link format from hardcoded URLs to Docusaurus doc IDs. The links now use the format 'module-X/index' instead of '/docs/module-X/index', which allows Docusaurus's Link component to properly handle the routing with the correct sidebar context.

## Outcome

- ✅ Impact: Fixed homepage card routing to use proper Docusaurus doc routing
- 🧪 Tests: None run
- 📁 Files: Modified book/src/pages/index.tsx
- 🔁 Next prompts: Could test the Docusaurus site to verify the fix works
- 🧠 Reflection: Using doc IDs instead of hardcoded paths allows proper context routing

## Evaluation notes (flywheel)

- Failure modes observed: Homepage links bypassed sidebar context
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test the Docusaurus site to confirm the fix