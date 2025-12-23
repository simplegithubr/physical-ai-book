"""
Main FastAPI application for the RAG chatbot system.
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from src.api.health import router as health_router
from src.config import config
import logging


# Configure logging
logging.basicConfig(
    level=getattr(logging, config.log_level.upper()),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for application startup and shutdown.
    """
    # Startup
    logger.info("Starting RAG Chatbot API...")

    # Initialize database connections, etc. here if needed
    # For now, we'll just log that the app is starting

    yield  # Application runs here

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")


# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API for Book Content Queries",
    description="API for querying Physical AI book content using Retrieval-Augmented Generation",
    version="1.0.0",
    lifespan=lifespan
)


# Add CORS middleware for Docusaurus integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for rate limiting
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining"]
)


# Include API routes
app.include_router(health_router, prefix="", tags=["health"])

from src.api.chat import router as chat_router
from src.api.ingest import router as ingest_router
from src.api.widget import router as widget_router

app.include_router(chat_router, prefix="", tags=["chat"])
app.include_router(ingest_router, prefix="", tags=["ingest"])
app.include_router(widget_router, prefix="", tags=["widget"])


@app.get("/")
async def root():
    """
    Root endpoint for basic API information.
    """
    return {
        "message": "RAG Chatbot API for Book Content Queries",
        "version": app.version,
        "status": "running",
        "debug": config.debug
    }


@app.get("/info")
async def info():
    """
    Information endpoint with configuration details (excluding sensitive data).
    """
    return {
        "title": app.title,
        "description": app.description,
        "version": app.version,
        "debug": config.debug,
        "log_level": config.log_level,
        "qdrant_collection": config.qdrant_collection_name,
        "models": {
            "embedding": config.embedding_model,
            "generation": config.generation_model
        }
    }


# Error handlers
# @app.exception_handler(404)
# async def not_found_handler(request, exc):
#     """
#     Handle 404 errors.
#     """
#     return HTTPException(status_code=404, detail="Endpoint not found")


# @app.exception_handler(500)
# async def internal_error_handler(request, exc):
#     """
#     Handle 500 errors.
#     """
#     logger.error(f"Internal server error: {exc}")
#     return HTTPException(status_code=500, detail="Internal server error")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=config.debug,
        log_level=config.log_level
    )