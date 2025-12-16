---
id: "3"
title: "Docusaurus Routing Analysis"
stage: "general"
date: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "003-isaac-robotics"
user: "unknown"
command: "/sp.analyze"
labels: ["docusaurus", "routing", "sidebar", "homepage", "configuration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "book/docusaurus.config.ts"
  - "book/sidebars.ts"
  - "book/src/pages/index.tsx"
  - "book/docs/module-1-ros2/index.md"
  - "book/docs/module2/index.md"
  - "book/docs/module-3-ai-robot-brain/index.md"
  - "book/docs/module-4-vla/index.md"
tests: []
---

## Prompt

Analyze Docusaurus routing, sidebar, and homepage configuration.

Project context:
- Docusaurus dev server runs successfully.
- Homepage shows 4 module cards (Module 1–4).
- Clicking any module card leads to "Page Not Found".
- Navbar Module 1–4 links work correctly.
- Tutorial sidebar shows only Module 1 instead of all 4.
- Blog and AI/Spec Book Creation sections are not desired.

Files involved:
- docusaurus.config.ts
- sidebars.ts
- src/pages/index.tsx (homepage cards)

Analysis goals:
1. Identify mismatch between homepage card links and actual doc IDs/slugs.
2. Verify sidebar IDs and doc paths for all modules.
3. Check Tutorial sidebar configuration and why it only includes one module.
4. Detect any incorrect baseUrl, routeBasePath, or doc entry misconfiguration.
5. Recommend exact fixes without implementing them.

Output required:
- Clear list of root causes.
- File-by-file diagnosis.
- Exact doc IDs and correct routes for Module 1–4.
- No code changes yet (analysis only).

## Response snapshot

## Docusaurus Configuration Analysis Report

### Root Causes Identified:

1. **Homepage Card Links Issue**: The homepage cards link to `/docs/module-X/index` paths, which should work since the corresponding index.md files exist. However, the issue might be that these module paths are not properly configured in the main docs sidebar, causing Docusaurus to not recognize them as valid doc paths when accessed directly from the homepage.

2. **Tutorial Sidebar Issue**: The tutorial sidebar is configured to show all 4 modules, but it may not be rendering properly because there's no main "intro" document that serves as the root of the tutorial sidebar. The tutorial sidebar expects a path like `/docs/intro` to exist as the starting point, but the actual modules are separate.

3. **Unwanted Blog Section**: The blog is enabled in `docusaurus.config.ts` presets, which creates unwanted blog functionality.

4. **Unwanted AI/Spec Book Creation Section**: There's an `aiSpecDrivenBookCreation` sidebar defined in `sidebars.ts` that may be referenced somewhere in the configuration.

### File-by-File Diagnosis:

**docusaurus.config.ts:**
- Blog is enabled (lines 49-63) - causing unwanted blog section
- Navbar configuration is correct for module links
- Sidebar path correctly set to './sidebars.ts'

**sidebars.ts:**
- `tutorialSidebar` has all 4 modules configured but may not render properly without a root doc
- Individual module sidebars (`module1Sidebar`, etc.) are correctly configured
- `aiSpecDrivenBookCreation` sidebar exists but may be unused

**src/pages/index.tsx:**
- Homepage card links are correctly formatted as `/docs/module-X/index`
- Module paths match the actual directory structure

### Exact Doc IDs and Correct Routes for Module 1–4:

- **Module 1**: `module-1-ros2/index` → `/docs/module-1-ros2/index`
- **Module 2**: `module2/index` → `/docs/module2/index`
- **Module 3**: `module-3-ai-robot-brain/index` → `/docs/module-3-ai-robot-brain/index`
- **Module 4**: `module-4-vla/index` → `/docs/module-4-vla/index`

### Recommended Fixes:

1. **Fix Homepage Card Links**: Add the module paths to a main docs sidebar or ensure the tutorial sidebar properly renders all categories.

2. **Fix Tutorial Sidebar**: Either add a main intro document that connects to the tutorial sidebar, or restructure the tutorial sidebar to properly show all 4 modules.

3. **Remove Unwanted Blog Section**: Disable the blog in `docusaurus.config.ts` by removing the blog configuration from the presets.

4. **Remove Unwanted AI/Spec Section**: Remove the `aiSpecDrivenBookCreation` sidebar from `sidebars.ts` if not needed.

5. **Ensure Proper Routing**: Make sure all module paths are properly recognized by Docusaurus by verifying the docs configuration in `docusaurus.config.ts`.

## Outcome

- ✅ Impact: Identified root causes of Docusaurus routing and sidebar issues
- 🧪 Tests: None run
- 📁 Files: Analyzed 3 key configuration files and 4 module index files
- 🔁 Next prompts: Could implement fixes based on recommendations
- 🧠 Reflection: The issue is likely that homepage links bypass the sidebar structure which navbar links use

## Evaluation notes (flywheel)

- Failure modes observed: Homepage card links lead to "Page Not Found" despite files existing
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Verify if homepage links work when accessed through proper sidebar structure