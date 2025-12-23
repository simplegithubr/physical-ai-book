"""
Ingest API endpoint for the RAG chatbot system.
Handles book content ingestion requests.
"""
from fastapi import APIRouter, HTTPException, Request
from typing import Dict, Any
import uuid
from pydantic import BaseModel, Field
from src.services.ingestion_service import BookIngestionService
from src.utils.rate_limit import rate_limit_middleware, add_rate_limit_headers
from src.utils.logging import get_logger
from src.utils.errors import RAGChatbotError, DatabaseConnectionError
import time
import asyncio


router = APIRouter()
logger = get_logger(__name__)
ingestion_service = BookIngestionService()


class IngestRequest(BaseModel):
    """
    Model for ingestion requests.
    """
    source_url: str = Field(..., min_length=1, max_length=1000)
    force_reindex: bool = Field(default=False)


class IngestResponse(BaseModel):
    """
    Model for ingestion responses.
    """
    status: str  # started, completed, failed
    job_id: str
    message: str
    source_url: str
    chunks_processed: int = 0
    duration_seconds: float = 0.0
    force_reindex: bool = False


class TextIngestRequest(BaseModel):
    """
    Model for text ingestion requests.
    """
    text: str = Field(..., min_length=1, max_length=100000)
    source_metadata: Dict[str, Any] = Field(default_factory=dict)


@router.post("/ingest", summary="Trigger book content ingestion")
async def trigger_ingestion(
    request: Request,
    ingest_request: IngestRequest
) -> IngestResponse:
    """
    Starts the process of scraping and indexing book content from the source URL.

    Args:
        request: FastAPI request object
        ingest_request: Ingestion request containing source URL and options

    Returns:
        IngestResponse with ingestion status
    """
    # Apply rate limiting
    rate_limit_response = rate_limit_middleware(request)
    if rate_limit_response:
        return rate_limit_response

    # Log the incoming request
    job_id = str(uuid.uuid4())
    logger.info(
        f"Ingestion request received",
        extra={
            "job_id": job_id,
            "source_url": ingest_request.source_url
        }
    )

    start_time = time.time()

    try:
        # Perform ingestion
        result = await ingestion_service.ingest_book_from_url(
            source_url=ingest_request.source_url,
            force_reindex=ingest_request.force_reindex
        )

        # Calculate duration
        duration = time.time() - start_time

        # Create response
        response = IngestResponse(
            status="completed",
            job_id=job_id,
            message="Ingestion completed successfully",
            source_url=ingest_request.source_url,
            chunks_processed=result.get("chunks_processed", 0),
            duration_seconds=round(duration, 2),
            force_reindex=ingest_request.force_reindex
        )

        # Log successful ingestion
        logger.info(
            f"Ingestion completed",
            extra={
                "job_id": job_id,
                "duration": duration,
                "chunks_processed": result.get("chunks_processed", 0)
            }
        )

        # Convert the Pydantic response to a JSONResponse to add headers
        from fastapi.responses import JSONResponse
        response_dict = response.dict()
        json_response = JSONResponse(content=response_dict)

        # Add rate limit headers to response
        add_rate_limit_headers(json_response, request.state)

        return json_response

    except DatabaseConnectionError as e:
        logger.error(f"Database error during ingestion: {str(e)}", extra={"job_id": job_id})
        raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")

    except ValueError as e:
        logger.warning(f"Invalid ingestion request: {str(e)}", extra={"job_id": job_id})
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        logger.error(f"Unexpected error during ingestion: {str(e)}", extra={"job_id": job_id}, exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/ingest/status/{job_id}", summary="Get ingestion job status")
async def get_ingestion_status(job_id: str) -> Dict[str, Any]:
    """
    Get the status of an ingestion job.

    Args:
        job_id: ID of the ingestion job

    Returns:
        Dict with job status information
    """
    # For now, we'll return a simple status
    # In a real implementation, you might track job status in a database
    return {
        "job_id": job_id,
        "status": "completed",  # Simplified for this example
        "message": "Job completed successfully",
        "completed_at": time.time()
    }


@router.get("/ingest/service-status", summary="Get ingestion service status")
async def get_service_status() -> Dict[str, Any]:
    """
    Get the current status of the ingestion service.

    Returns:
        Dict with service status information
    """
    try:
        status = ingestion_service.get_ingestion_status()
        return {
            "status": "healthy" if status["service_ready"] else "degraded",
            "details": status
        }
    except Exception as e:
        logger.error(f"Error getting service status: {str(e)}", exc_info=True)
        return {
            "status": "unavailable",
            "error": str(e),
            "details": {}
        }


@router.post("/ingest/text", summary="Ingest content from raw text")
async def ingest_from_text(
    request: Request,
    ingest_request: TextIngestRequest
) -> IngestResponse:
    """
    Ingest book content from raw text.

    Args:
        request: FastAPI request object
        ingest_request: Request containing text and metadata

    Returns:
        IngestResponse with ingestion status
    """
    # Apply rate limiting
    rate_limit_response = rate_limit_middleware(request)
    if rate_limit_response:
        return rate_limit_response

    # Log the incoming request
    job_id = str(uuid.uuid4())
    logger.info(
        f"Text ingestion request received",
        extra={
            "job_id": job_id,
            "text_length": len(ingest_request.text)
        }
    )

    start_time = time.time()

    try:
        # Perform text ingestion
        result = await ingestion_service.ingest_from_text(
            text=ingest_request.text,
            source_metadata=ingest_request.source_metadata or {"url": "text_input", "source_type": "manual_text_entry"}
        )

        # Calculate duration
        duration = time.time() - start_time

        # Create response
        response = IngestResponse(
            status="completed",
            job_id=job_id,
            message="Text ingestion completed successfully",
            source_url=ingest_request.source_metadata.get("url", "text_input"),
            chunks_processed=result.get("chunks_processed", 0),
            duration_seconds=round(duration, 2)
        )

        # Log successful ingestion
        logger.info(
            f"Text ingestion completed",
            extra={
                "job_id": job_id,
                "duration": duration,
                "chunks_processed": result.get("chunks_processed", 0)
            }
        )

        # Convert the Pydantic response to a JSONResponse to add headers
        from fastapi.responses import JSONResponse
        response_dict = response.dict()
        json_response = JSONResponse(content=response_dict)

        # Add rate limit headers to response
        add_rate_limit_headers(json_response, request.state)

        return json_response

    except Exception as e:
        logger.error(f"Unexpected error during text ingestion: {str(e)}", extra={"job_id": job_id}, exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")



# Example usage
if __name__ == "__main__":
    import asyncio

    async def test_ingestion():
        # Example ingestion request
        request_data = IngestRequest(
            source_url="https://example.com/book"
        )

        try:
            result = await ingestion_service.ingest_book_from_url(request_data.source_url)
            print(f"Ingestion result: {result}")
        except Exception as e:
            print(f"Error: {e}")

    # Note: This example won't run without proper API keys configured
    # asyncio.run(test_ingestion())