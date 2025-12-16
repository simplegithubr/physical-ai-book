<!--
Sync Impact Report:
- Version change: N/A -> 1.0.0
- Modified principles: N/A (new constitution)
- Added sections: All principles and sections
- Removed sections: None
- Templates requiring updates: N/A
- Follow-up TODOs: None
-->

# Book + Embedded RAG Chatbot for Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical Accuracy
All implementations must maintain technical accuracy with ROS 2, Gazebo, Unity, Isaac, VLA, and LLMs; Solutions must be verified using official documentation and reproduceable in real environments

### Student Clarity
All content and code must be designed for clarity to CS/AI students; Complex concepts must be explained with tutorials, diagrams, and workflows that are easy to follow

### Reproducible Code
All code snippets and setups must be reproducible; Every example must be tested and verified to work in the intended environment

### Source Verification
All technical information must be verified using official documentation and authoritative sources before inclusion

### Complete Integration
The book and RAG chatbot must work as a complete integrated system covering all 4 modules: ROS 2 Nervous System, Digital Twin, NVIDIA Isaac AI-Robot Brain, and Vision-Language-Action

### No Hallucination Guarantee
The RAG chatbot must answer only from book content or user-selected text with zero hallucinations; Strict adherence to source material is mandatory

## Technology Stack Standards
Book built with Docusaurus, deployed on GitHub Pages, written using Spec-Kit Plus + Claude Code; RAG chatbot uses OpenAI Agents/ChatKit, FastAPI, NeonDB, Qdrant; All examples use ROS 2, Python, Isaac, Nav2, Whisper

## Development Workflow
All code must be tested with verified examples; Each module must include tutorials, diagrams, workflows, and simulations; End-to-end testing required for humanoid workflow: voice → plan → ROS 2 actions → simulation

## Governance
All PRs/reviews must verify compliance with technical accuracy and reproducibility; Code must be tested in actual environments; Book content must be validated against official documentation; RAG chatbot must pass accuracy tests with zero hallucinations

**Version**: 1.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12
