"""
Logging configuration utilities for the RAG chatbot system.
Provides structured logging for debugging and monitoring.
"""
import logging
from typing import Dict, Any, Optional
from datetime import datetime
import json
from src.config import config


class JSONFormatter(logging.Formatter):
    """
    Custom JSON formatter for structured logging.
    """
    def format(self, record: logging.LogRecord) -> str:
        log_entry = {
            'timestamp': datetime.utcfromtimestamp(record.created).isoformat() + 'Z',
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno
        }

        # Add exception info if present
        if record.exc_info:
            log_entry['exception'] = self.formatException(record.exc_info)

        # Add extra fields if present
        if hasattr(record, 'user_id'):
            log_entry['user_id'] = record.user_id
        if hasattr(record, 'request_id'):
            log_entry['request_id'] = record.request_id
        if hasattr(record, 'query'):
            log_entry['query'] = record.query
        if hasattr(record, 'response_time'):
            log_entry['response_time'] = record.response_time

        return json.dumps(log_entry)


def setup_logging() -> None:
    """
    Setup logging configuration for the application.
    """
    # Create a custom logger
    logger = logging.getLogger()
    logger.setLevel(getattr(logging, config.log_level.upper()))

    # Remove default handlers to avoid duplicate logs
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # Create console handler with a higher log level
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, config.log_level.upper()))

    # Create formatter and add it to the handler
    if config.debug:
        # Use a more readable format for debugging
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        # Use JSON format for production
        formatter = JSONFormatter()

    console_handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(console_handler)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.

    Args:
        name: Name of the logger

    Returns:
        logging.Logger: Configured logger instance
    """
    return logging.getLogger(name)


def log_api_call(
    logger: logging.Logger,
    endpoint: str,
    method: str,
    user_id: Optional[str] = None,
    query: Optional[str] = None,
    response_time: Optional[float] = None,
    status_code: Optional[int] = None
) -> None:
    """
    Log an API call with relevant information.

    Args:
        logger: Logger instance to use
        endpoint: API endpoint that was called
        method: HTTP method used
        user_id: ID of the user making the call
        query: Query string or request content
        response_time: Time taken to process the request
        status_code: HTTP status code of the response
    """
    extra = {
        'user_id': user_id,
        'query': query,
        'response_time': response_time,
        'status_code': status_code
    }

    logger.info(
        f"API call: {method} {endpoint}",
        extra=extra
    )


def log_error(
    logger: logging.Logger,
    error: Exception,
    context: Optional[Dict[str, Any]] = None
) -> None:
    """
    Log an error with context information.

    Args:
        logger: Logger instance to use
        error: Exception that occurred
        context: Additional context information
    """
    extra = {'error_type': type(error).__name__}
    if context:
        extra.update(context)

    logger.error(
        f"Error occurred: {str(error)}",
        extra=extra,
        exc_info=True  # Include traceback
    )


def log_performance(
    logger: logging.Logger,
    operation: str,
    duration: float,
    details: Optional[Dict[str, Any]] = None
) -> None:
    """
    Log performance metrics for an operation.

    Args:
        logger: Logger instance to use
        operation: Name of the operation
        duration: Duration of the operation in seconds
        details: Additional details about the operation
    """
    extra = {
        'operation': operation,
        'duration': duration
    }
    if details:
        extra.update(details)

    logger.info(
        f"Performance: {operation} took {duration:.3f}s",
        extra=extra
    )


# Setup logging when module is imported
setup_logging()


# Example usage
if __name__ == "__main__":
    # Get a logger for this module
    logger = get_logger(__name__)

    # Log different types of events
    logger.info("Application started")
    logger.warning("This is a warning message")

    # Log an API call
    log_api_call(
        logger=logger,
        endpoint="/chat",
        method="POST",
        user_id="user123",
        query="What is quantum computing?",
        response_time=1.234,
        status_code=200
    )

    # Log performance
    log_performance(
        logger=logger,
        operation="vector_search",
        duration=0.456,
        details={"query_length": 100, "results_count": 5}
    )

    # Log an error
    try:
        raise ValueError("Something went wrong")
    except ValueError as e:
        log_error(
            logger=logger,
            error=e,
            context={"user_id": "user123", "operation": "query_processing"}
        )