"""
ChatRequest model for the RAG chatbot system.
Model for incoming chat requests.
"""
from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
from datetime import datetime


class Message(BaseModel):
    """
    Model for a single message in conversation history.
    """
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str = Field(..., min_length=1, max_length=5000)


class ChatRequest(BaseModel):
    """
    Model for incoming chat requests.
    """
    query: str = Field(..., min_length=1, max_length=2000)
    selected_text: Optional[str] = Field(default=None, max_length=5000)
    history: List[Message] = Field(default=[], max_items=10)
    book_id: str = Field(default="physical-ai")
    temperature: Optional[float] = Field(default=None, ge=0.0, le=1.0)

    @validator('book_id')
    def validate_book_id(cls, v):
        if v != "physical-ai":
            raise ValueError('book_id must be "physical-ai"')
        return v

    @validator('query')
    def validate_query(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('query cannot be empty')
        return v

    @validator('temperature')
    def validate_temperature(cls, v):
        if v is not None and (v < 0.0 or v > 1.0):
            raise ValueError('temperature must be between 0.0 and 1.0')
        return v


class ChatRequestResponse(BaseModel):
    """
    Response model for chat request validation.
    """
    query: str
    selected_text: Optional[str]
    history_count: int
    book_id: str
    temperature: Optional[float]
    processed_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    @classmethod
    def from_request(cls, request: ChatRequest) -> 'ChatRequestResponse':
        return cls(
            query=request.query,
            selected_text=request.selected_text,
            history_count=len(request.history),
            book_id=request.book_id,
            temperature=request.temperature,
            processed_at=datetime.utcnow()
        )


# Example usage
if __name__ == "__main__":
    # Example of creating a ChatRequest
    chat_request = ChatRequest(
        query="What are the key principles of Physical AI?",
        selected_text=None,
        history=[
            Message(role="user", content="Explain Physical AI"),
            Message(role="assistant", content="Physical AI is a field that combines...")
        ],
        book_id="physical-ai",
        temperature=0.7
    )

    print("Created ChatRequest:")
    print(f"  Query: {chat_request.query}")
    print(f"  Selected text: {chat_request.selected_text}")
    print(f"  History count: {len(chat_request.history)}")
    print(f"  Book ID: {chat_request.book_id}")
    print(f"  Temperature: {chat_request.temperature}")

    # Example of validation error
    try:
        invalid_request = ChatRequest(
            query="",
            book_id="invalid-book"
        )
    except ValueError as e:
        print(f"Validation error: {e}")

    # Example with selected text
    selected_text_request = ChatRequest(
        query="What does this text mean?",
        selected_text="Physical AI combines robotics with artificial intelligence to create systems that can interact with the physical world intelligently.",
        book_id="physical-ai"
    )

    print(f"\nRequest with selected text: {selected_text_request.selected_text[:50]}...")

    # Example of creating response model
    response = ChatRequestResponse.from_request(chat_request)
    print(f"\nResponse model: {response.dict()}")