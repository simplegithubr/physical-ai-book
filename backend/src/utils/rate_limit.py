"""
Rate limiting utilities for the RAG chatbot system.
Provides API rate limiting functionality to respect free tier limits.
"""
from typing import Dict, Optional, Tuple
from datetime import datetime, timedelta
import time
from fastapi import Request
from fastapi.responses import JSONResponse
from src.config import config
import logging


logger = logging.getLogger(__name__)


class RateLimiter:
    """
    Simple in-memory rate limiter to track API usage.
    """
    def __init__(self, requests_per_minute: int = 100):
        """
        Initialize the rate limiter.

        Args:
            requests_per_minute: Maximum requests allowed per minute
        """
        self.requests_per_minute = requests_per_minute
        self.requests: Dict[str, list] = {}  # ip -> [timestamps]
        self.lockout_until: Dict[str, datetime] = {}  # ip -> datetime

    def is_allowed(self, identifier: str) -> Tuple[bool, int, int]:
        """
        Check if a request from the given identifier is allowed.

        Args:
            identifier: Unique identifier (e.g., IP address)

        Returns:
            Tuple of (is_allowed, remaining_requests, reset_time_seconds)
        """
        now = datetime.now()

        # Check if IP is currently locked out
        if identifier in self.lockout_until:
            if now < self.lockout_until[identifier]:
                # Still in lockout period
                remaining_time = int((self.lockout_until[identifier] - now).total_seconds())
                return False, 0, remaining_time
            else:
                # Lockout period expired, remove from lockout
                del self.lockout_until[identifier]

        # Clean old requests (older than 1 minute)
        one_minute_ago = now - timedelta(minutes=1)
        if identifier in self.requests:
            self.requests[identifier] = [
                timestamp for timestamp in self.requests[identifier]
                if timestamp > one_minute_ago
            ]
        else:
            self.requests[identifier] = []

        # Check if we've exceeded the limit
        current_requests = len(self.requests[identifier])
        if current_requests >= self.requests_per_minute:
            # Calculate when the lockout should end (1 minute from first request in window)
            first_request_time = self.requests[identifier][0]
            lockout_end = first_request_time + timedelta(minutes=1)
            self.lockout_until[identifier] = lockout_end

            remaining_time = int((lockout_end - now).total_seconds())
            return False, 0, remaining_time

        # Add current request timestamp
        self.requests[identifier].append(now)

        # Calculate remaining requests and reset time
        remaining = self.requests_per_minute - current_requests - 1
        reset_time = int((self.requests[identifier][0] + timedelta(minutes=1) - now).total_seconds())

        return True, remaining, reset_time

    def get_usage_stats(self, identifier: str) -> Dict[str, int]:
        """
        Get usage statistics for the given identifier.

        Args:
            identifier: Unique identifier (e.g., IP address)

        Returns:
            Dict with usage statistics
        """
        now = datetime.now()
        one_minute_ago = now - timedelta(minutes=1)

        if identifier in self.requests:
            recent_requests = [
                timestamp for timestamp in self.requests[identifier]
                if timestamp > one_minute_ago
            ]
            current_usage = len(recent_requests)
        else:
            current_usage = 0

        return {
            "current_usage": current_usage,
            "limit": self.requests_per_minute,
            "remaining": max(0, self.requests_per_minute - current_usage)
        }


# Global rate limiter instance
rate_limiter = RateLimiter(requests_per_minute=60)  # Default to 60 requests per minute


def rate_limit_middleware(request: Request) -> Optional[JSONResponse]:
    """
    Middleware function to check rate limits.

    Args:
        request: FastAPI request object

    Returns:
        JSONResponse if rate limit exceeded, None otherwise
    """
    if config.debug:
        # Skip rate limiting in debug mode
        return None

    # Get client IP address
    client_ip = request.client.host if request.client else "unknown"

    # Check if request is allowed
    is_allowed, remaining, reset_time = rate_limiter.is_allowed(client_ip)

    # Add rate limit headers to response
    request.state.rate_limit_remaining = remaining
    request.state.rate_limit_reset = reset_time

    if not is_allowed:
        return JSONResponse(
            status_code=429,
            content={
                "error": "Rate limit exceeded",
                "message": f"Too many requests from this IP. Try again in {reset_time} seconds."
            },
            headers={
                "X-RateLimit-Limit": str(rate_limiter.requests_per_minute),
                "X-RateLimit-Remaining": str(0),
                "X-RateLimit-Reset": str(reset_time)
            }
        )

    return None


def add_rate_limit_headers(response, request_state) -> None:
    """
    Add rate limit headers to the response.

    Args:
        response: Response object to add headers to
        request_state: Request state containing rate limit info
    """
    # Safely get rate limit values using getattr to handle missing attributes
    remaining = getattr(request_state, 'rate_limit_remaining', 0)
    reset_time = getattr(request_state, 'rate_limit_reset', 60)

    response.headers["X-RateLimit-Limit"] = str(rate_limiter.requests_per_minute)
    response.headers["X-RateLimit-Remaining"] = str(remaining)
    response.headers["X-RateLimit-Reset"] = str(reset_time)


def check_rate_limit(identifier: str) -> Tuple[bool, Dict[str, int]]:
    """
    Check rate limit for an identifier and return detailed information.

    Args:
        identifier: Unique identifier to check

    Returns:
        Tuple of (is_allowed, stats_dict)
    """
    is_allowed, remaining, reset_time = rate_limiter.is_allowed(identifier)
    stats = rate_limiter.get_usage_stats(identifier)
    stats["remaining"] = remaining
    stats["reset_time"] = reset_time

    return is_allowed, stats


# Example usage and testing
if __name__ == "__main__":
    # Test the rate limiter
    print("Testing rate limiter...")

    # Test with a single identifier
    identifier = "127.0.0.1"

    # Make several requests
    for i in range(3):
        is_allowed, remaining, reset_time = rate_limiter.is_allowed(identifier)
        print(f"Request {i+1}: Allowed={is_allowed}, Remaining={remaining}, Reset in {reset_time}s")

    # Check usage stats
    stats = rate_limiter.get_usage_stats(identifier)
    print(f"Usage stats: {stats}")

    # Simulate time passing
    print("\nSimulating time passing...")
    import asyncio

    async def test_rate_limit():
        # Clear the requests for this test
        rate_limiter.requests[identifier] = []

        # Make requests up to the limit
        for i in range(rate_limiter.requests_per_minute):
            is_allowed, remaining, reset_time = rate_limiter.is_allowed(identifier)
            if i % 20 == 0:  # Print every 20th request
                print(f"Request {i+1}: Allowed={is_allowed}, Remaining={remaining}")

        # Next request should be denied
        is_allowed, remaining, reset_time = rate_limiter.is_allowed(identifier)
        print(f"Request {rate_limiter.requests_per_minute + 1}: Allowed={is_allowed}, Remaining={remaining}, Reset in {reset_time}s")

    asyncio.run(test_rate_limit())