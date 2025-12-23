# Research Findings: Integrated RAG Chatbot Development for Book Content Queries

## Overview
This document captures research findings for the RAG chatbot implementation, addressing unknowns from the Technical Context and establishing best practices for the chosen technologies.

## Technology Research

### Cohere API Integration
- **Decision**: Use Cohere's `embed-english-v3.0` for embeddings and `command-r-plus` for generation
- **Rationale**: Aligns with constitution requirement to use Cohere APIs exclusively; v3.0 embeddings offer improved performance and multilingual support
- **Alternatives considered**: OpenAI embeddings (rejected due to constitution requirement), Hugging Face models (rejected for consistency with Cohere ecosystem)
- **Best practices**:
  - Implement proper rate limiting to respect free tier limits
  - Use asynchronous calls for better performance
  - Cache embeddings to reduce API costs

### Qdrant Vector Database
- **Decision**: Use Qdrant cloud cluster with provided configuration
- **Rationale**: Offers efficient similarity search capabilities needed for RAG system; supports Cohere embeddings well
- **Best practices**:
  - Configure proper collection schema for book content chunks
  - Set up point payloads with metadata (URL, section, etc.)
  - Implement proper error handling for network issues

### FastAPI Backend Framework
- **Decision**: Use FastAPI for backend API
- **Rationale**: High-performance, async-ready, excellent for ML/AI applications; automatic OpenAPI docs
- **Best practices**:
  - Use Pydantic models for request/response validation
  - Implement proper error handling and logging
  - Configure CORS for Docusaurus integration

### Book Content Scraping
- **Decision**: Build custom scraper using BeautifulSoup4
- **Rationale**: Docusaurus sites have predictable structure; allows control over content extraction
- **Best practices**:
  - Respect robots.txt and rate limiting
  - Extract semantic structure (headings, paragraphs, code blocks)
  - Preserve metadata (URL, section hierarchy)

## Architecture Decisions

### Embedding Strategy
- **Decision**: Chunk size of 600-800 tokens with 200-token overlap
- **Rationale**: Balances context retention with retrieval precision; respects section boundaries
- **Implementation**: Use token-based chunking with boundary awareness

### API Design
- **Decision**: RESTful API with POST /chat endpoint as primary interface
- **Rationale**: Simple, well-understood pattern; supports both full-text and selected-text queries
- **Considerations**: Implement proper request validation, response streaming for better UX

## Security Considerations
- **Decision**: Secure API keys in environment variables, never commit to repo
- **Rationale**: Meets constitution requirement for secure handling of sensitive information
- **Implementation**: Use python-dotenv, configure git to ignore .env files

## Performance Targets
- **Decision**: Target <5s response time for all queries
- **Rationale**: Meets constitution performance requirement
- **Implementation**: Optimize vector search, implement caching, use async operations

## Unknowns Resolved
- Language/version: Python 3.11 for backend, vanilla JS/HTML/CSS for frontend
- Primary dependencies: FastAPI, Cohere, Qdrant-client, BeautifulSoup4 confirmed
- Storage: Qdrant for vector storage, Neon Postgres for metadata confirmed
- Testing: pytest for backend testing confirmed
- Target platform: Linux server backend, cross-browser frontend widget confirmed
- Performance goals: <5s response time confirmed
- Constraints: Cohere API limits, free tier usage limits confirmed
- Scale/scope: 100 concurrent users target confirmed