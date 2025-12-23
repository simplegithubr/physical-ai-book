# Quickstart Guide: Integrated RAG Chatbot for Book Content Queries

## Overview
This guide provides quick setup instructions for the RAG chatbot system that enables querying Physical AI book content.

## Prerequisites
- Python 3.11+
- `uv` package manager
- Access to Cohere API key
- Qdrant vector database access
- Neon Postgres database access

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Backend Environment
```bash
cd backend
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file in the backend directory with the following:
```env
COHERE_API_KEY="your_cohere_api_key"
QDRANT_URL="https://your-qdrant-cluster-url:6333"
QDRANT_API_KEY="your_qdrant_api_key"
NEON_DATABASE_URL="your_neon_database_url"
```

### 4. Install Dependencies
```bash
uv pip install -r requirements.txt
```

### 5. Run the Application
```bash
uvicorn src.main:app --reload
```

## API Usage

### Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "query": "What are the key principles of Physical AI?",
    "book_id": "physical-ai"
  }'
```

### Ingestion Endpoint
```bash
curl -X POST http://localhost:8000/ingest \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "source_url": "https://book.physical-ai.com"
  }'
```

## Frontend Integration

### Embedding the Widget
Add the following iframe to your Docusaurus page:
```html
<iframe
  src="http://localhost:8000/widget"
  width="400"
  height="600"
  frameborder="0">
</iframe>
```

## Running Tests
```bash
pytest
```

## Development Commands
- `uvicorn src.main:app --reload` - Run the development server
- `python ingestion/scripts/book_scraper.py` - Run the book content scraper
- `pytest` - Run the test suite
- `pytest -v` - Run tests with verbose output