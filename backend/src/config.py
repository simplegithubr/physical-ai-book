"""
Configuration loader for the RAG chatbot system.
Loads configuration from environment variables with validation.
"""
import os
from typing import Optional
from pydantic import BaseModel, validator
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


class Config(BaseModel):
    """
    Application configuration model with validation.
    """
    # API Keys
    cohere_api_key: str
    qdrant_api_key: str
    neon_database_url: str

    # API Endpoints
    qdrant_url: str

    # Application settings
    api_key: str
    debug: bool = False
    log_level: str = "info"

    # Model settings
    embedding_model: str = "embed-english-v3.0"
    generation_model: str = "command-r-08-2024"  # Updated to current model as command-r-plus was removed

    # Qdrant settings
    qdrant_collection_name: str = "physical_ai_book"

    # Performance settings
    max_chunk_size: int = 700
    overlap_size: int = 200
    max_query_results: int = 8
    response_timeout: int = 30  # seconds

    @validator('cohere_api_key')
    def validate_cohere_api_key(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('COHERE_API_KEY must be set and not empty')
        return v

    @validator('qdrant_api_key')
    def validate_qdrant_api_key(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('QDRANT_API_KEY must be set and not empty')
        return v

    @validator('neon_database_url')
    def validate_neon_database_url(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('NEON_DATABASE_URL must be set and not empty')
        if not v.startswith('postgresql://'):
            raise ValueError('NEON_DATABASE_URL must be a valid PostgreSQL URL')
        return v

    @validator('qdrant_url')
    def validate_qdrant_url(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('QDRANT_URL must be set and not empty')
        if not (v.startswith('http://') or v.startswith('https://')):
            raise ValueError('QDRANT_URL must be a valid URL')
        return v


def get_config() -> Config:
    """
    Load and validate configuration from environment variables.

    Returns:
        Config: Validated configuration object
    """
    config = Config(
        cohere_api_key=os.getenv("COHERE_API_KEY", ""),
        qdrant_api_key=os.getenv("QDRANT_API_KEY", ""),
        neon_database_url=os.getenv("NEON_DATABASE_URL", ""),
        qdrant_url=os.getenv("QDRANT_URL", ""),
        api_key=os.getenv("API_KEY", "default-api-key-for-testing"),
        debug=os.getenv("DEBUG", "false").lower() == "true",
        log_level=os.getenv("LOG_LEVEL", "info"),
    )

    return config


# Global config instance
config = get_config()


if __name__ == "__main__":
    # Example usage
    print("Configuration loaded successfully:")
    print(f"  Debug mode: {config.debug}")
    print(f"  Log level: {config.log_level}")
    print(f"  Qdrant collection: {config.qdrant_collection_name}")
    print(f"  Max chunk size: {config.max_chunk_size}")