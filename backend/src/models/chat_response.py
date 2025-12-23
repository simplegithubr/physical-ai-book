"""
ChatResponse model for the RAG chatbot system.
Model for outgoing chat responses.
"""
from pydantic import BaseModel, Field, validator
from typing import List, Optional
from datetime import datetime
import uuid


class Source(BaseModel):
    """
    Model for source citations in chat responses.
    """
    section: str = Field(..., min_length=1, max_length=500)
    excerpt: str = Field(..., min_length=1, max_length=1000)
    url: str = Field(..., min_length=1, max_length=1000)
    similarity_score: float = Field(..., ge=0.0, le=1.0)

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


class ChatResponse(BaseModel):
    """
    Model for outgoing chat responses.
    """
    response: str = Field(..., min_length=1)
    sources: List[Source] = Field(default=[], max_items=8)
    query_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    @validator('response')
    def validate_response(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('response cannot be empty')
        return v

    @validator('sources')
    def validate_sources(cls, v):
        if len(v) > 8:
            raise ValueError('sources cannot contain more than 8 items')
        return v


class ChatResponseMetadata(BaseModel):
    """
    Additional metadata for chat responses.
    """
    query_id: str
    response_time_ms: int
    token_count: int
    model_used: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


# Example usage
if __name__ == "__main__":
    # Example of creating a ChatResponse
    sources = [
        Source(
            section="Introduction to Physical AI",
            excerpt="Physical AI is defined as the intersection of robotics and AI...",
            url="https://book.physical-ai.com/docs/introduction",
            similarity_score=0.85
        ),
        Source(
            section="Key Principles",
            excerpt="The core principles include embodied intelligence and physical interaction...",
            url="https://book.physical-ai.com/docs/principles",
            similarity_score=0.78
        )
    ]

    chat_response = ChatResponse(
        response="Physical AI combines robotics with artificial intelligence to create systems that can interact with the physical world intelligently...",
        sources=sources,
        query_id="chat-12345"
    )

    print("Created ChatResponse:")
    print(f"  Query ID: {chat_response.query_id}")
    print(f"  Response: {chat_response.response[:50]}...")
    print(f"  Sources count: {len(chat_response.sources)}")
    print(f"  Timestamp: {chat_response.timestamp}")

    for i, source in enumerate(chat_response.sources):
        print(f"  Source {i+1}: {source.section} ({source.similarity_score})")

    # Example of validation error
    try:
        invalid_response = ChatResponse(
            response="",
            sources=[
                Source(
                    section="",
                    excerpt="",
                    url="invalid-url",
                    similarity_score=1.5
                )
            ]
        )
    except ValueError as e:
        print(f"Validation error: {e}")

    # Example of creating response metadata
    metadata = ChatResponseMetadata(
        query_id=chat_response.query_id,
        response_time_ms=1250,
        token_count=45,
        model_used="command-r-plus"
    )
    print(f"\nResponse metadata: {metadata.dict()}")