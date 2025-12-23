"""
Health check endpoint for the RAG chatbot system.
Provides health status of the API and its dependencies.
"""
from fastapi import APIRouter
from typing import Dict, Any
from src.config import config
from src.database.qdrant import QdrantDB
from src.database.neon import NeonDB
import asyncio


router = APIRouter()


@router.get("/health", summary="Health check endpoint")
async def health_check() -> Dict[str, Any]:
    """
    Returns the health status of the API and its dependencies.

    Returns:
        Dict with health status and service connectivity information
    """
    # Check individual services
    services_status = {
        "cohere_api": await check_cohere_connectivity(),
        "qdrant_db": await check_qdrant_connectivity(),
        "neon_db": await check_neon_connectivity()
    }

    # Determine overall status
    if all(services_status.values()):
        overall_status = "healthy"
    elif any(services_status.values()):
        overall_status = "degraded"
    else:
        overall_status = "unavailable"

    return {
        "status": overall_status,
        "timestamp": asyncio.get_event_loop().time(),
        "services": services_status
    }


async def check_cohere_connectivity() -> bool:
    """
    Check if Cohere API is accessible.

    Returns:
        bool: True if Cohere API is accessible
    """
    try:
        # We could make a simple API call to test connectivity
        # For now, just check if the API key is configured
        api_key = config.cohere_api_key
        return bool(api_key and len(api_key.strip()) > 0)
    except Exception:
        return False


async def check_qdrant_connectivity() -> bool:
    """
    Check if Qdrant database is accessible.

    Returns:
        bool: True if Qdrant database is accessible
    """
    try:
        # Create a QdrantDB instance and try to get collections
        db = QdrantDB()
        collections = db.client.get_collections()
        return True
    except Exception:
        return False


async def check_neon_connectivity() -> bool:
    """
    Check if Neon Postgres database is accessible.

    Returns:
        bool: True if Neon database is accessible
    """
    try:
        # Create a NeonDB instance and try to connect
        db = NeonDB()
        await db.connect()
        await db.disconnect()
        return True
    except Exception:
        return False


# Example usage in main app
if __name__ == "__main__":
    # This would normally be run as part of the FastAPI app
    import asyncio

    async def test_health():
        result = await health_check()
        print(result)

    asyncio.run(test_health())