"""
Embedding generation module for the RAG chatbot system.
Implements embedding generation using Cohere's embed-english-v3.0 model.
"""
import os
import asyncio
from typing import List, Dict, Any
from cohere import Client
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


class Embedder:
    """
    Handles embedding generation using Cohere API.
    """

    def __init__(self, model: str = "embed-english-v3.0"):
        """
        Initialize the embedder with the specified model.

        Args:
            model: Cohere embedding model to use
        """
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = Client(api_key=api_key)
        self.model = model

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input for the embedding model

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        if not texts:
            return []

        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type=input_type
        )

        return response.embeddings

    def embed_single_text(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input for the embedding model

        Returns:
            Embedding as a list of floats
        """
        embeddings = self.embed_texts([text], input_type)
        return embeddings[0] if embeddings else []

    async def embed_texts_async(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Asynchronously generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input for the embedding model

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        if not texts:
            return []

        # Note: The Cohere Python SDK is synchronous, so we simulate async behavior
        # In a real implementation, you might use aiohttp to call the API directly
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.embed_texts, texts, input_type)


# Example usage
if __name__ == "__main__":
    # This would require a valid COHERE_API_KEY in the environment
    try:
        embedder = Embedder()

        sample_texts = [
            "This is the first text to embed.",
            "This is the second text for embedding.",
            "Finally, a third text sample."
        ]

        embeddings = embedder.embed_texts(sample_texts)
        print(f"Generated {len(embeddings)} embeddings")
        print(f"First embedding has {len(embeddings[0])} dimensions")

        single_embedding = embedder.embed_single_text("Just a single text")
        print(f"Single embedding has {len(single_embedding)} dimensions")

    except ValueError as e:
        print(f"Error: {e}")
        print("Please set your COHERE_API_KEY environment variable to test embedding functionality.")