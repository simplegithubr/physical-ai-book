# Data Model: Integrated RAG Chatbot Development for Book Content Queries

## Overview
This document defines the data models for the RAG chatbot system, including entities for book content, user interactions, and system metadata.

## Core Entities

### Book Content Models

#### BookChunk
Represents a chunk of book content stored in the vector database.

```json
{
  "chunk_id": "str",
  "book_id": "str",
  "text_content": "str",
  "metadata": {
    "url": "str",
    "section_title": "str",
    "chapter": "str",
    "level": "int",
    "position": "int"
  },
  "embedding": "list[float]", // Cohere embedding vector
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

**Validation Rules:**
- chunk_id: Unique identifier, required
- book_id: Fixed value "physical-ai", required
- text_content: Max 800 tokens, required
- metadata.url: Valid URL format, required
- embedding: 1024-dimensional vector (for embed-english-v3.0)

#### BookMetadata
Stores metadata about the indexed book content in Neon Postgres.

```sql
CREATE TABLE book_metadata (
  id SERIAL PRIMARY KEY,
  book_id VARCHAR(255) UNIQUE NOT NULL,
  version VARCHAR(50),
  total_chunks INTEGER DEFAULT 0,
  last_ingested TIMESTAMP WITH TIME ZONE,
  total_pages INTEGER DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

### API Request/Response Models

#### ChatRequest
Model for incoming chat requests.

```json
{
  "query": "str",
  "selected_text": "str | null",
  "history": [
    {
      "role": "user | assistant",
      "content": "str"
    }
  ],
  "book_id": "str",
  "temperature": "float | null"
}
```

**Validation Rules:**
- query: Required, min length 1, max length 2000 characters
- selected_text: Optional, max length 5000 characters
- history: Max 10 conversation turns
- book_id: Must be "physical-ai"
- temperature: 0.0 to 1.0 if provided

#### ChatResponse
Model for outgoing chat responses.

```json
{
  "response": "str",
  "sources": [
    {
      "section": "str",
      "excerpt": "str",
      "url": "str",
      "similarity_score": "float"
    }
  ],
  "query_id": "str",
  "timestamp": "datetime"
}
```

**Validation Rules:**
- response: Required, non-empty
- sources: Array of 0-8 items
- similarity_score: 0.0 to 1.0

#### IngestRequest
Model for triggering book content ingestion.

```json
{
  "source_url": "str",
  "force_reindex": "bool | null"
}
```

**Validation Rules:**
- source_url: Required, valid URL format
- force_reindex: Optional, defaults to false

### System Models

#### HealthCheck
Health check response model.

```json
{
  "status": "healthy | degraded | unavailable",
  "timestamp": "datetime",
  "services": {
    "cohere_api": "bool",
    "qdrant_db": "bool",
    "neon_db": "bool"
  }
}
```

## Relationships

### Book Content Relationships
- BookMetadata (1) : BookChunk (Many) - One book metadata record relates to many content chunks
- BookChunk (1) : Sources in ChatResponse (Many) - One chunk can be cited in multiple responses

### API Flow Relationships
- ChatRequest (1) : ChatResponse (1) - One request generates one response
- ChatRequest (Many) : BookChunk (Many) - Many requests can retrieve from many chunks

## State Transitions

### Book Ingestion States
- `PENDING` → `PROCESSING` → `COMPLETED` | `FAILED`
- Stored in BookMetadata.last_ingested field to indicate the last successful ingestion

## Indexing Strategy

### Qdrant Collection Schema
```python
collection_config = {
    "name": "physical_ai_book",
    "vector_size": 1024,  // Cohere embed-english-v3.0
    "distance": "Cosine",
    "payload_schema": {
        "chunk_text": "keyword",
        "chunk_id": "keyword",
        "url": "keyword",
        "section": "keyword",
        "book_id": "keyword"
    }
}
```

### Neon Postgres Indexes
```sql
-- Book metadata indexes
CREATE INDEX idx_book_metadata_book_id ON book_metadata(book_id);
CREATE INDEX idx_book_metadata_last_ingested ON book_metadata(last_ingested);

-- Future user interaction indexes (if needed)
-- CREATE INDEX idx_interactions_query_time ON interactions(timestamp);
```

## Validation Rules Summary

### Content Accuracy
- All response text must be traceable to BookChunk content
- Source citations required in responses
- Zero tolerance for hallucinated content

### Performance
- BookChunk text_content limited to prevent oversized embeddings
- Efficient indexing for fast retrieval
- API response time under 5 seconds

### Security
- No user data stored persistently
- API keys stored securely in environment variables
- Input validation on all endpoints