"""
BookMetadata model for the RAG chatbot system.
Stores metadata about the indexed book content in Neon Postgres.
"""
from pydantic import BaseModel, Field, validator
from typing import Optional
from datetime import datetime


class BookMetadata(BaseModel):
    """
    Model representing metadata about a book.
    """
    id: Optional[int] = Field(default=None)
    book_id: str = Field(default="physical-ai")
    version: Optional[str] = Field(default=None, max_length=50)
    total_chunks: int = Field(default=0, ge=0)
    last_ingested: Optional[datetime] = Field(default=None)
    total_pages: int = Field(default=0, ge=0)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    @validator('book_id')
    def validate_book_id(cls, v):
        if v != "physical-ai":
            raise ValueError('book_id must be "physical-ai"')
        return v

    @validator('total_chunks', 'total_pages')
    def validate_non_negative(cls, v):
        if v < 0:
            raise ValueError('Value must be non-negative')
        return v

    class Config:
        # Enable ORM mode for database integration
        from_attributes = True
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class BookMetadataCreateRequest(BaseModel):
    """
    Request model for creating book metadata.
    """
    book_id: str = Field(default="physical-ai")
    version: Optional[str] = Field(default=None, max_length=50)
    total_pages: int = Field(default=0, ge=0)

    @validator('book_id')
    def validate_book_id(cls, v):
        if v != "physical-ai":
            raise ValueError('book_id must be "physical-ai"')
        return v

    @validator('total_pages')
    def validate_total_pages(cls, v):
        if v < 0:
            raise ValueError('total_pages must be non-negative')
        return v


class BookMetadataUpdateRequest(BaseModel):
    """
    Request model for updating book metadata.
    """
    version: Optional[str] = Field(default=None, max_length=50)
    total_chunks: Optional[int] = Field(default=None, ge=0)
    total_pages: Optional[int] = Field(default=None, ge=0)

    @validator('total_chunks', 'total_pages')
    def validate_non_negative(cls, v):
        if v is not None and v < 0:
            raise ValueError('Value must be non-negative')
        return v


class BookMetadataResponse(BaseModel):
    """
    Response model for book metadata operations.
    """
    id: int
    book_id: str
    version: Optional[str]
    total_chunks: int
    last_ingested: Optional[datetime]
    total_pages: int
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
    # Example of creating BookMetadata
    metadata = BookMetadata(
        book_id="physical-ai",
        version="1.0.0",
        total_chunks=150,
        total_pages=300,
        last_ingested=datetime.utcnow()
    )

    print("Created BookMetadata:")
    print(f"  ID: {metadata.id}")
    print(f"  Book ID: {metadata.book_id}")
    print(f"  Version: {metadata.version}")
    print(f"  Total Chunks: {metadata.total_chunks}")
    print(f"  Total Pages: {metadata.total_pages}")
    print(f"  Last Ingested: {metadata.last_ingested}")

    # Example of creating a request
    create_request = BookMetadataCreateRequest(
        version="1.0.1",
        total_pages=320
    )
    print(f"\nCreate request: {create_request.dict()}")

    # Example of updating a request
    update_request = BookMetadataUpdateRequest(
        total_chunks=160,
        total_pages=320
    )
    print(f"Update request: {update_request.dict()}")

    # Example of validation error
    try:
        invalid_metadata = BookMetadata(
            book_id="invalid-book",
            total_chunks=-5
        )
    except ValueError as e:
        print(f"Validation error: {e}")