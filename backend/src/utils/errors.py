"""
Error handling utilities for the RAG chatbot system.
Provides custom exceptions and error handling patterns.
"""
from typing import Optional, Dict, Any
from fastapi import HTTPException
from fastapi.responses import JSONResponse
import logging


logger = logging.getLogger(__name__)


class RAGChatbotError(Exception):
    """
    Base exception class for RAG Chatbot system.
    """
    def __init__(self, message: str, error_code: Optional[str] = None, details: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert error to dictionary format for API responses.
        """
        return {
            "error": self.message,
            "error_code": self.error_code,
            "details": self.details
        }


class ContentNotFoundError(RAGChatbotError):
    """
    Raised when requested content is not found in the knowledge base.
    """
    def __init__(self, message: str = "Requested content not found in the knowledge base", query: Optional[str] = None):
        super().__init__(
            message=message,
            error_code="CONTENT_NOT_FOUND",
            details={"query": query} if query else {}
        )


class EmbeddingGenerationError(RAGChatbotError):
    """
    Raised when there's an error generating embeddings.
    """
    def __init__(self, message: str = "Error generating embeddings", original_error: Optional[Exception] = None):
        super().__init__(
            message=message,
            error_code="EMBEDDING_GENERATION_ERROR",
            details={"original_error": str(original_error)} if original_error else {}
        )


class DatabaseConnectionError(RAGChatbotError):
    """
    Raised when there's an error connecting to the database.
    """
    def __init__(self, message: str = "Database connection error", db_type: Optional[str] = None):
        super().__init__(
            message=message,
            error_code="DATABASE_CONNECTION_ERROR",
            details={"database_type": db_type} if db_type else {}
        )


class APIRateLimitError(RAGChatbotError):
    """
    Raised when API rate limits are exceeded.
    """
    def __init__(self, message: str = "API rate limit exceeded", reset_time: Optional[str] = None):
        super().__init__(
            message=message,
            error_code="API_RATE_LIMIT_EXCEEDED",
            details={"reset_time": reset_time} if reset_time else {}
        )


class InvalidQueryError(RAGChatbotError):
    """
    Raised when a query is invalid or malformed.
    """
    def __init__(self, message: str = "Invalid query provided", query: Optional[str] = None):
        super().__init__(
            message=message,
            error_code="INVALID_QUERY",
            details={"query": query} if query else {}
        )


class ConfigurationError(RAGChatbotError):
    """
    Raised when there's a configuration issue.
    """
    def __init__(self, message: str = "Configuration error", config_key: Optional[str] = None):
        super().__init__(
            message=message,
            error_code="CONFIGURATION_ERROR",
            details={"config_key": config_key} if config_key else {}
        )


def handle_error(error: Exception, log_error: bool = True) -> RAGChatbotError:
    """
    Convert an exception to an appropriate RAGChatbotError.

    Args:
        error: The original exception
        log_error: Whether to log the error

    Returns:
        RAGChatbotError: Appropriate error for the system
    """
    if log_error:
        logger.error(f"Error handled: {type(error).__name__}: {str(error)}", exc_info=True)

    # Map common exceptions to our custom errors
    if isinstance(error, RAGChatbotError):
        return error
    elif isinstance(error, HTTPException):
        return RAGChatbotError(
            message=f"HTTP error: {error.detail}",
            error_code=f"HTTP_{error.status_code}",
            details={"status_code": error.status_code}
        )
    elif isinstance(error, ValueError):
        return InvalidQueryError(message=str(error))
    elif isinstance(error, KeyError):
        return ConfigurationError(message=f"Missing configuration: {str(error)}")
    else:
        return RAGChatbotError(
            message=f"Unexpected error: {str(error)}",
            error_code="UNEXPECTED_ERROR",
            details={"original_error_type": type(error).__name__}
        )


def create_error_response(error: RAGChatbotError, status_code: int = 500) -> JSONResponse:
    """
    Create a JSON response for an error.

    Args:
        error: The RAGChatbotError to convert
        status_code: HTTP status code for the response

    Returns:
        JSONResponse: Formatted error response
    """
    return JSONResponse(
        status_code=status_code,
        content=error.to_dict()
    )


# Example usage
if __name__ == "__main__":
    # Example of raising and handling different errors
    try:
        # Simulate a content not found scenario
        raise ContentNotFoundError("Could not find information about quantum computing in the provided content", "quantum computing")
    except RAGChatbotError as e:
        print(f"Caught RAGChatbotError: {e.message}")
        print(f"Error code: {e.error_code}")
        print(f"Details: {e.details}")
        print(f"Dict format: {e.to_dict()}")

    # Example of handling a generic exception
    try:
        raise ValueError("Invalid input provided")
    except Exception as e:
        handled_error = handle_error(e)
        print(f"Handled error: {handled_error.message}")
        print(f"Error code: {handled_error.error_code}")