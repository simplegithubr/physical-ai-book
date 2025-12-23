"""
BookChunk model for the RAG chatbot system.
Represents a chunk of book content stored in the vector database.
"""
from pydantic import BaseModel, Field, validator
from typing import Dict, Any, Optional
from datetime import datetime
import uuid


class BookChunk(BaseModel):
    """
    Model representing a chunk of book content.
    """
    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    book_id: str = Field(default="physical-ai")
    text_content: str = Field(..., min_length=1, max_length=5000)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    embedding: Optional[list] = Field(default=None)  # Will be populated when embedding is generated
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    @validator('book_id')
    def validate_book_id(cls, v):
        if v != "physical-ai":
            raise ValueError('book_id must be "physical-ai"')
        return v

    @validator('text_content')
    def validate_text_content(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('text_content cannot be empty')
        return v

    @validator('metadata')
    def validate_metadata(cls, v):
        # Ensure required metadata fields exist
        required_fields = ['url']
        for field in required_fields:
            if field not in v:
                v[field] = ""
        return v

    class Config:
        # Allow extra fields but ignore them
        extra = "allow"
        # Enable ORM mode for database integration
        from_attributes = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class BookChunkCreateRequest(BaseModel):
    """
    Request model for creating a book chunk.
    """
    text_content: str = Field(..., min_length=1, max_length=5000)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    book_id: str = Field(default="physical-ai")

    @validator('book_id')
    def validate_book_id(cls, v):
        if v != "physical-ai":
            raise ValueError('book_id must be "physical-ai"')
        return v

    @validator('text_content')
    def validate_text_content(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('text_content cannot be empty')
        return v


class BookChunkResponse(BaseModel):
    """
    Response model for book chunk operations.
    """
    chunk_id: str
    book_id: str
    text_content: str
    metadata: Dict[str, Any]
    created_at: datetime
    updated_at: datetime

    class Config:
        # Enable ORM mode for database integration
        from_attributes = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


# Example usage
if __name__ == "__main__":
    # Example of creating a BookChunk
    chunk = BookChunk(
        text_content="This is a sample book content chunk for testing purposes.",
        metadata={
            "url": "http://example.com/chapter1",
            "section": "Introduction",
            "chapter": "Chapter 1"
        }
    )

    print("Created BookChunk:")
    print(f"  ID: {chunk.chunk_id}")
    print(f"  Content: {chunk.text_content[:50]}...")
    print(f"  Metadata: {chunk.metadata}")
    print(f"  Book ID: {chunk.book_id}")

    # Example of validating input
    try:
        invalid_chunk = BookChunk(
            text_content="",
            metadata={"url": "http://example.com"}
        )
    except ValueError as e:
        print(f"Validation error: {e}")

    # Example of creating a request
    request = BookChunkCreateRequest(
        text_content="This is content from a book chapter.",
        metadata={
            "url": "http://example.com/chapter2",
            "section": "Advanced Topics"
        }
    )
    print(f"\nRequest model: {request.dict()}")