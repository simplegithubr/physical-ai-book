<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Modified principles: All principles replaced with RAG-focused principles
- Added sections: New principles for RAG chatbot project
- Removed sections: Previous ROS 2/NVIDIA Isaac principles
- Templates requiring updates: ✅ Updated
- Follow-up TODOs: None
-->

# Integrated RAG Chatbot Development for Book Content Queries Constitution

## Core Principles

### Content Accuracy and Context-Awareness
All responses must be generated using book content only when specified; Responses must be contextually relevant and technically accurate to the source material; Zero hallucinations allowed when referencing book content.

### Resource Efficiency
Leverage free tiers without exceeding limits; Optimize API usage and computational resources; Maintain cost-effective operations while preserving quality and performance.

### User-Centric Design
Design intuitive interfaces for queries and text selection; Prioritize usability and accessibility in all user interactions; Ensure seamless experience for users querying book content.

### Security and Privacy
No storage of user data beyond sessions; Secure API keys and sensitive information; Protect user privacy through secure handling of queries and responses.

## Technology Stack Standards

All responses must be generated using Cohere APIs exclusively (no OpenAI fallback); Code modularity: Separate concerns (e.g., embedding pipeline, API endpoints, frontend UI); Tech stack: Cohere API, FastAPI, Neon Serverless Postgres, Qdrant; Testing rigor: Unit tests for retrieval accuracy (e.g., via cosine similarity thresholds in Qdrant) and integration tests for end-to-end flow.

## Development Workflow

All code must include proper documentation: Inline comments and README with setup instructions, including Cloud IDE deployment; Performance targets: Response time under 5 seconds; Scalability: Handle up to 100 concurrent users on free tiers; End-to-end testing required for retrieval-augmented generation flow.

## Governance

All PRs/reviews must verify compliance with content accuracy and resource efficiency; Code must be tested with proper unit and integration tests; Book content queries must pass accuracy tests with strict adherence to source material; RAG chatbot must maintain performance benchmarks and security standards.

**Version**: 1.1.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-19