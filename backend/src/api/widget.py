"""
Widget API endpoint for the RAG chatbot system.
Provides embeddable widget functionality.
"""
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from typing import Dict, Any
import os
from src.utils.rate_limit import rate_limit_middleware, add_rate_limit_headers
from src.utils.logging import get_logger


router = APIRouter()
logger = get_logger(__name__)

# Serve static files for the widget
frontend_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "frontend")
router.mount("/static", StaticFiles(directory=os.path.join(frontend_path, "static")), name="static")


@router.get("/widget", summary="Get embeddable chat widget", response_class=HTMLResponse)
async def get_widget(request: Request) -> HTMLResponse:
    """
    Returns the HTML for the embeddable chat widget.

    Args:
        request: FastAPI request object

    Returns:
        HTMLResponse with the widget HTML
    """
    # Apply rate limiting
    rate_limit_response = rate_limit_middleware(request)
    if rate_limit_response:
        return rate_limit_response

    try:
        # Read the widget HTML
        widget_html_path = os.path.join(frontend_path, "static", "index.html")

        with open(widget_html_path, "r", encoding="utf-8") as file:
            html_content = file.read()

        # Customize the HTML with request-specific information
        # Add API base URL and other configuration
        api_base_url = str(request.base_url).rstrip("/")
        html_content = html_content.replace("</head>", f"""
    <script>
      // Configuration for the chat widget
      window.CHAT_CONFIG = {{
        apiBaseUrl: "{api_base_url}"
      }};

      // Set data attributes for the API base URL
      document.addEventListener('DOMContentLoaded', function() {{
        const container = document.getElementById('chat-container');
        if (container) {{
          container.dataset.api_base = "{api_base_url}";
        }}
      }});
    </script>
  </head>""")

        # Log widget access
        logger.info("Widget requested", extra={"client_ip": request.client.host if request.client else "unknown"})

        # Create response
        response = HTMLResponse(content=html_content)

        # Add security headers for iframe (Task T50: Implement iframe security headers)
        response.headers["Content-Security-Policy"] = "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'self' http://localhost:* https://*.vercel.app https://*.github.io;"
        response.headers["X-Frame-Options"] = "SAMEORIGIN"
        response.headers["X-Content-Type-Options"] = "nosniff"

        # Add rate limit headers to response
        add_rate_limit_headers(response, request.state)

        return response

    except FileNotFoundError:
        logger.error("Widget HTML file not found")
        raise HTTPException(status_code=500, detail="Widget files not found")
    except Exception as e:
        logger.error(f"Error serving widget: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/widget/config", summary="Get widget configuration")
async def get_widget_config(request: Request) -> Dict[str, Any]:
    """
    Returns the configuration for the chat widget.

    Args:
        request: FastAPI request object

    Returns:
        Dict with widget configuration
    """
    # Apply rate limiting
    rate_limit_response = rate_limit_middleware(request)
    if rate_limit_response:
        return rate_limit_response

    config = {
        "api_base_url": str(request.base_url).rstrip("/"),
        "version": "1.0.0",
        "features": {
            "text_selection": True,
            "source_citations": True,
            "history": False
        },
        "limits": {
            "max_query_length": 2000,
            "max_selected_text_length": 5000
        }
    }

    # Convert the dict response to a JSONResponse to add headers
    from fastapi.responses import JSONResponse
    json_response = JSONResponse(content=config)

    # Add rate limit headers to response
    add_rate_limit_headers(json_response, request.state)

    return json_response


@router.get("/widget/health", summary="Widget service health check")
async def widget_health_check(request: Request) -> Dict[str, Any]:
    """
    Health check endpoint for the widget service.

    Args:
        request: FastAPI request object

    Returns:
        Dict with health status
    """
    # Apply rate limiting
    rate_limit_response = rate_limit_middleware(request)
    if rate_limit_response:
        return rate_limit_response

    # Check if widget files are available
    widget_files_available = True
    try:
        widget_html_path = os.path.join(frontend_path, "static", "index.html")
        if not os.path.exists(widget_html_path):
            widget_files_available = False
    except:
        widget_files_available = False

    status = {
        "status": "healthy" if widget_files_available else "degraded",
        "details": {
            "frontend_files_available": widget_files_available,
            "static_files_mounted": True
        }
    }

    # Convert the dict response to a JSONResponse to add headers
    from fastapi.responses import JSONResponse
    json_response = JSONResponse(content=status)

    # Add rate limit headers to response
    add_rate_limit_headers(json_response, request.state)

    return json_response


# Additional endpoint for frontend assets
@router.get("/widget/assets/{file_path:path}", summary="Serve widget assets")
async def serve_widget_asset(file_path: str):
    """
    Serves widget assets like CSS, JS, and images.

    Args:
        file_path: Path to the asset file

    Returns:
        FileResponse with the requested asset
    """
    asset_path = os.path.join(frontend_path, "static", file_path)

    if not os.path.exists(asset_path):
        raise HTTPException(status_code=404, detail="Asset not found")

    return FileResponse(asset_path)



# Update the main app to include CORS configuration (Task T51: Add CORS configuration for embedding)
# This is handled in main.py, but we'll add a reference here
def add_cors_middleware(app):
    """
    This function is referenced here but implemented in main.py
    to add CORS configuration for embedding in various domains.
    """
    pass