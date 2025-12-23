# Physical AI & Humanoid Robotics Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-18

## Active Technologies

- Python 3.11
- Cohere API for embeddings and generation (embed-english-v3.0, command-r-plus)
- Qdrant vector database
- FastAPI for backend API
- Requests for HTTP requests
- BeautifulSoup4 for HTML parsing
- python-dotenv for environment management
- uv for package management
- pdfplumber for PDF processing (fallback)
- Neon Serverless Postgres for metadata storage
- pytest for testing

## Project Structure

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── main.py          # FastAPI application entry point
├── requirements.txt     # Dependencies: fastapi, cohere, qdrant-client, requests, beautifulsoup4, python-dotenv
├── .env                 # Environment variables for API keys
└── tests/
    ├── unit/
    └── integration/

frontend/
├── static/
│   ├── js/
│   ├── css/
│   └── index.html
└── assets/

ingestion/
├── scripts/
│   └── book_scraper.py
├── chunker.py
└── embedder.py

tests/
├── contract/
├── integration/
└── unit/

docs/
└── integration-guide.md
```

## Commands

- `uv venv` - Create virtual environment
- `source .venv/bin/activate` - Activate virtual environment (Windows: `.venv\Scripts\activate`)
- `uv pip install -r requirements.txt` - Install dependencies
- `uvicorn backend.src.main:app --reload` - Run the FastAPI backend server
- `python ingestion/scripts/book_scraper.py` - Run the book content scraper
- `pytest` - Run the test suite

## Code Style

- Use Python 3.11+ features appropriately
- Follow PEP 8 guidelines
- Include type hints for function parameters and return values
- Use meaningful variable names
- Include error handling for external API calls and file operations
- Use environment variables for sensitive information (API keys)

## Recent Changes

- RAG Chatbot Backend: Implemented FastAPI backend with chat and ingestion endpoints
- Book Content Ingestion: Created scraping pipeline to extract content from Docusaurus sites
- Vector Storage: Integrated Qdrant for efficient similarity search of book content
- Cohere Integration: Implemented embed-english-v3.0 for embeddings and command-r-plus for generation
- Frontend Widget: Designed embeddable chat interface for Docusaurus integration
- API Contracts: Defined OpenAPI specification for the RAG system

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->