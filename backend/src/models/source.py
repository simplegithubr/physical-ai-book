"""
Source model for the RAG chatbot system.
Model for source citations in chat responses.
"""
from pydantic import BaseModel, Field, validator
from typing import Optional
from datetime import datetime


class Source(BaseModel):
    """
    Model for source citations in chat responses.
    """
    section: str = Field(..., min_length=1, max_length=500)
    excerpt: str = Field(..., min_length=1, max_length=1000)
    url: str = Field(..., min_length=1, max_length=1000)
    similarity_score: float = Field(..., ge=0.0, le=1.0)
    chunk_id: Optional[str] = Field(default=None)
    page_number: Optional[int] = Field(default=None, ge=1)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    @validator('url')
    def validate_url(cls, v):
        if not v.startswith(('http://', 'https://')):
            raise ValueError('url must be a valid URL starting with http:// or https://')
        return v

    @validator('similarity_score')
    def validate_similarity_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('similarity_score must be between 0.0 and 1.0')
        return v

    @validator('excerpt')
    def validate_excerpt(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('excerpt cannot be empty')
        return v

    @validator('section')
    def validate_section(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('section cannot be empty')
        return v

    class Config:
        # Enable ORM mode for database integration
        from_attributes = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class SourceCreateRequest(BaseModel):
    """
    Request model for creating a source.
    """
    section: str = Field(..., min_length=1, max_length=500)
    excerpt: str = Field(..., min_length=1, max_length=1000)
    url: str = Field(..., min_length=1, max_length=1000)
    similarity_score: float = Field(..., ge=0.0, le=1.0)
    chunk_id: Optional[str] = Field(default=None)
    page_number: Optional[int] = Field(default=None, ge=1)

    @validator('url')
    def validate_url(cls, v):
        if not v.startswith(('http://', 'https://')):
            raise ValueError('url must be a valid URL starting with http:// or https://')
        return v

    @validator('similarity_score')
    def validate_similarity_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('similarity_score must be between 0.0 and 1.0')
        return v


class SourceResponse(BaseModel):
    """
    Response model for source operations.
    """
    section: str
    excerpt: str
    url: str
    similarity_score: float
    chunk_id: Optional[str]
    page_number: Optional[int]
    created_at: datetime

    class Config:
        # Enable ORM mode for database integration
        from_attributes = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


# Example usage
if __name__ == "__main__":
    # Example of creating a Source
    source = Source(
        section="Introduction to Physical AI",
        excerpt="Physical AI is defined as the intersection of robotics and AI...",
        url="https://book.physical-ai.com/docs/introduction",
        similarity_score=0.85,
        chunk_id="chunk-12345",
        page_number=5
    )

    print("Created Source:")
    print(f"  Section: {source.section}")
    print(f"  Excerpt: {source.excerpt[:50]}...")
    print(f"  URL: {source.url}")
    print(f"  Similarity Score: {source.similarity_score}")
    print(f"  Chunk ID: {source.chunk_id}")
    print(f"  Page Number: {source.page_number}")

    # Example of validation error
    try:
        invalid_source = Source(
            section="",
            excerpt="",
            url="invalid-url",
            similarity_score=1.5
        )
    except ValueError as e:
        print(f"Validation error: {e}")

    # Example of creating a request
    request = SourceCreateRequest(
        section="Key Principles",
        excerpt="The core principles include embodied intelligence and physical interaction...",
        url="https://book.physical-ai.com/docs/principles",
        similarity_score=0.78
    )
    print(f"\nRequest model: {request.dict()}")

    # Example of creating a response
    response = SourceResponse(
        section=source.section,
        excerpt=source.excerpt,
        url=source.url,
        similarity_score=source.similarity_score,
        chunk_id=source.chunk_id,
        page_number=source.page_number,
        created_at=source.created_at
    )
    print(f"Response model: {response.dict()}")