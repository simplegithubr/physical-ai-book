---
id: 1
title: "Fix Docusaurus Homepage Module Card Routing"
stage: general
dateIso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "003-isaac-robotics"
user: "user"
command: "/sp.implement"
labels: ["docusaurus", "routing", "homepage", "fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "book/src/pages/index.tsx"
tests_yaml: []
---

## Prompt Text

```
You are working on a Docusaurus v2 documentation site.

PROBLEM:
- Navbar links for Module 1–4 work correctly
- Homepage module cards open "Page Not Found"
- Docs files and sidebars exist and are correct
- Blog is intentionally disabled
- Only 4 modules should exist

ROOT CAUSE TO INVESTIGATE:
Homepage cards link directly to relative paths (e.g. "module-1-ros2/index")
which bypass the /docs routing context required by Docusaurus.

TASKS:
1. Analyze src/pages/index.tsx
2. Identify incorrect homepage Link routing
3. Fix module card links so they correctly open docs pages
4. Ensure links work with sidebar + navbar context
5. Use best-practice Docusaurus routing (docs base path)

IMPLEMENTATION RULES:
- Do NOT change sidebar structure
- Do NOT add blog
- Do NOT rename docs files
- Only fix homepage routing
- Prefer `/docs/<doc-id>` or useBaseUrl
- Result: homepage cards must open same pages as navbar

OUTPUT:
- Modify src/pages/index.tsx
- Provide clean, minimal fix
- Ensure all 4 module cards work
- No new fea
```

## Response Text

Successfully fixed the Docusaurus homepage module card routing issue by updating the link paths in `book/src/pages/index.tsx`. The problem was that homepage cards were using relative paths like `module-1-ros2/index` which bypassed the `/docs` routing context required by Docusaurus.

Changed all module card `to` properties to include the `/docs` prefix:
- `module-1-ros2/index` → `/docs/module-1-ros2/index`
- `module2/index` → `/docs/module2/index`
- `module-3-ai-robot-brain/index` → `/docs/module-3-ai-robot-brain/index`
- `module-4-vla/index` → `/docs/module-4-vla/index`

The fix ensures homepage cards now open the same pages as navbar links, following Docusaurus best practices for documentation routing.

## Outcome

The Docusaurus homepage module cards now correctly route to their respective documentation pages by using the proper `/docs/` prefixed paths that match the working footer and navbar links.
