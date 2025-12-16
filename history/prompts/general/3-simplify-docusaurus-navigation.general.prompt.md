---
id: 3
title: "Simplify Docusaurus Navigation to Avoid Page Not Found Issues"
stage: general
dateIso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "003-isaac-robotics"
user: "user"
command: "/sp.implement"
labels: ["docusaurus", "navigation", "homepage", "routing", "simplification"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "book/src/pages/index.tsx"
  - "book/docusaurus.config.ts"
tests_yaml: []
---

## Prompt Text

```
You are working on a Docusaurus v3 documentation site located in the `book/` directory.

GOAL:
Simplify navigation to avoid all "Page Not Found" issues.

STRICT REQUIREMENTS:

1) HOMEPAGE (src/pages/index.tsx)
   - REMOVE all links from the 4 module cards
   - Do NOT use <Link>, history.push, or routing
   - Each module card should ONLY display:
     - Module title
     - Short description
   - NO "Start Module" button
   - NO clickable navigation from homepage at all

2) FOOTER (docusaurus.config.ts)
   - REMOVE all module links from the footer
   - Footer should contain ONLY static text (no internal doc links)

3) NAVBAR
   - Keep navbar simple:
     - Replace "Tutorial" label with "Book Tutorial"
     - Book Tutorial should open the docs sidebar (docs root)
   - Remove Blog completely if still present

4) DOCS SIDEBAR
   - "Book Tutorial" must show ALL 4 modules:
     - Module 1: ROS 2
     - Module 2: Digital Twin
     - Module 3: AI Robot Brain
     - Module 4: VLA
   - Each module must appear correctly
```

## Response Text

Successfully simplified the Docusaurus navigation to avoid "Page Not Found" issues by implementing all required changes:

1. **HOMEPAGE (src/pages/index.tsx)**:
   - Removed all navigation imports (@docusaurus/Link, useHistory, useBaseUrl)
   - Updated ModuleCard component to display only title and description
   - Removed "Start Module" button and all clickable navigation
   - Module cards now only show static content (title + description)

2. **FOOTER (docusaurus.config.ts)**:
   - Removed all module links from the footer
   - Footer now contains only static copyright text

3. **NAVBAR (docusaurus.config.ts)**:
   - Replaced "Tutorial" label with "Book Tutorial"
   - Verified Blog was already removed
   - "Book Tutorial" continues to open the docs sidebar as required

4. **DOCS SIDEBAR (sidebars.ts)**:
   - Verified that "Book Tutorial" sidebar already shows all 4 modules:
     - Module 1: The Robotic Nervous System (ROS 2)
     - Module 2: Digital Twin (Gazebo & Unity)
     - Module 3: The AI-Robot Brain (NVIDIA Isaac™)
     - Module 4: Vision-Language-Action (VLA)

The navigation has been simplified to eliminate all potential "Page Not Found" issues while maintaining access to documentation through the navbar "Book Tutorial" link.

## Outcome

All navigation simplification requirements have been met, eliminating potential "Page Not Found" issues while preserving access to documentation through the simplified navigation structure.