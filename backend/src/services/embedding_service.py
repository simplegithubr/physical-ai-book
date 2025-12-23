"""
EmbeddingService for the RAG chatbot system.
Handles embedding generation using Cohere API and response generation.
"""
from typing import List, Optional, Dict, Any
import os
from cohere import Client
from dotenv import load_dotenv
from src.config import config
from src.utils.errors import EmbeddingGenerationError
from src.utils.logging import get_logger
import asyncio


# Load environment variables
load_dotenv()

logger = get_logger(__name__)


class EmbeddingService:
    """
    Service class for handling embeddings and text generation using Cohere.
    """

    def __init__(self):
        """
        Initialize the EmbeddingService with Cohere client.
        """
        api_key = config.cohere_api_key
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = Client(api_key=api_key)
        self.embedding_model = config.embedding_model
        self.generation_model = config.generation_model

    def embed_texts(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input for the embedding model

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            if not texts:
                return []

            response = self.client.embed(
                texts=texts,
                model=self.embedding_model,
                input_type=input_type
            )

            return response.embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}", exc_info=True)
            raise EmbeddingGenerationError(f"Failed to generate embeddings: {str(e)}", e)

    def embed_single_text(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed
            input_type: Type of input for the embedding model

        Returns:
            Embedding as a list of floats
        """
        try:
            embeddings = self.embed_texts([text], input_type)
            return embeddings[0] if embeddings else []

        except Exception as e:
            logger.error(f"Error generating embedding for single text: {str(e)}", exc_info=True)
            raise EmbeddingGenerationError(f"Failed to generate embedding: {str(e)}", e)

    def generate_response_with_rag(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        temperature: Optional[float] = None,
        preamble: Optional[str] = None
    ) -> str:
        """
        Generate a response using Cohere's Chat API with RAG (documents parameter).

        Args:
            query: The user's query
            context_chunks: Context chunks used for the response
            temperature: Temperature for response generation (0.0 to 1.0)
            preamble: Optional preamble to force the model to follow specific rules

        Returns:
            Generated response text
        """
        try:
            # Set default temperature if not provided
            if temperature is None:
                temperature = 0.7

            # Set default preamble to force the model to answer only from book content
            if preamble is None:
                preamble = (
                    "You are an AI assistant specialized in answering questions based ONLY on the provided context documents. "
                    "You must follow these rules strictly:\n"
                    "1. Answer only based on information provided in the documents below.\n"
                    "2. If the information is not in the documents, clearly state that the information is not available in the provided context.\n"
                    "3. Never make up, fabricate, or hallucinate information.\n"
                    "4. Never use external knowledge or general world knowledge.\n"
                    "5. Always cite relevant documents when providing information.\n"
                    "6. Be concise but comprehensive in your responses.\n"
                    "7. If asked about topics not covered in the documents, acknowledge that the information is not in the provided context."
                )

            # Prepare documents for RAG from context chunks
            documents = []
            for chunk in context_chunks:
                document = {
                    "title": chunk.get("section", "Unknown Section"),
                    "snippet": chunk.get("text_content", ""),
                    "url": chunk.get("url", "")
                }
                documents.append(document)

            # Generate response using Cohere Chat API with documents parameter
            response = self.client.chat(
                message=query,
                documents=documents,
                model=self.generation_model,
                temperature=temperature,
                max_tokens=500,  # Limit response length
                preamble=preamble,
                # Additional parameters for better RAG performance
                prompt_truncation='AUTO',
                connectors=[]  # Explicitly disable connectors to ensure only provided documents are used
            )

            # Extract the generated text
            if response and response.text:
                generated_text = response.text.strip()
                return generated_text
            else:
                raise EmbeddingGenerationError("No response generated by the model")

        except Exception as e:
            logger.error(f"Error generating response with Cohere Chat API: {str(e)}", exc_info=True)
            raise EmbeddingGenerationError(f"Failed to generate response: {str(e)}", e)

    def generate_response(
        self,
        prompt: str,
        query: str,
        context_chunks: List[Dict[str, Any]],
        temperature: Optional[float] = None
    ) -> str:
        """
        Generate a response using Cohere's command-r-plus model.
        This method is deprecated and now uses the new Chat API with RAG.

        Args:
            prompt: The formatted prompt to send to the model
            query: The original user query
            context_chunks: Context chunks used for the response
            temperature: Temperature for response generation (0.0 to 1.0)

        Returns:
            Generated response text
        """
        # This method is now deprecated - using the new Chat API with RAG
        return self.generate_response_with_rag(query, context_chunks, temperature)

    def rerank_results(
        self,
        query: str,
        results: List[Dict[str, Any]],
        top_n: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Rerank search results using Cohere's rerank functionality for better relevance.

        Args:
            query: The original query
            results: List of search results to rerank
            top_n: Number of top results to return

        Returns:
            Reranked list of results
        """
        try:
            # Extract text content from results for reranking
            documents = [result.get('text_content', '') for result in results]

            # Use Cohere's rerank API
            rerank_response = self.client.rerank(
                query=query,
                documents=documents,
                top_n=top_n
            )

            # Reorder results based on rerank scores
            reranked_results = []
            for ranked_doc in rerank_response.results:
                original_result = results[ranked_doc.index]
                original_result['rerank_score'] = ranked_doc.relevance_score
                original_result['similarity_score'] = max(
                    original_result.get('similarity_score', 0),
                    ranked_doc.relevance_score
                )
                reranked_results.append(original_result)

            return reranked_results

        except Exception as e:
            logger.warning(f"Reranking failed, returning original results: {str(e)}")
            # If reranking fails, return original results with original scores
            return results

    async def embed_texts_async(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Asynchronously generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            input_type: Type of input for the embedding model

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            # In a real implementation, this would use async/await with the Cohere API
            # For now, we'll just call the sync version in a thread pool
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(None, self.embed_texts, texts, input_type)
        except Exception as e:
            logger.error(f"Error generating embeddings asynchronously: {str(e)}", exc_info=True)
            raise EmbeddingGenerationError(f"Failed to generate embeddings: {str(e)}", e)

    def get_token_count(self, text: str) -> int:
        """
        Get approximate token count for a text.

        Args:
            text: Text to count tokens for

        Returns:
            Approximate token count
        """
        # This is a rough approximation - 1 token is roughly 4 characters or 0.75 words
        if not text:
            return 0

        # Split on whitespace and punctuation to get a better estimate
        import re
        tokens = re.findall(r'\b\w+\b|[^\w\s]', text)
        return len(tokens)


# Example usage
if __name__ == "__main__":
    # This example requires a valid COHERE_API_KEY in the environment
    try:
        service = EmbeddingService()

        # Test embedding generation
        sample_texts = [
            "This is the first text to embed.",
            "This is the second text for embedding.",
            "Finally, a third text sample."
        ]

        print("Testing embedding generation...")
        embeddings = service.embed_texts(sample_texts)
        print(f"Generated {len(embeddings)} embeddings")
        print(f"First embedding has {len(embeddings[0])} dimensions")

        single_embedding = service.embed_single_text("Just a single text")
        print(f"Single embedding has {len(single_embedding)} dimensions")

        # Test response generation
        print("\nTesting response generation...")
        prompt = "What is artificial intelligence? Answer in one sentence."
        response = service.generate_response(
            prompt=prompt,
            query="What is AI?",
            context_chunks=[]
        )
        print(f"Generated response: {response}")

        # Test token counting
        print(f"\nToken count for sample text: {service.get_token_count('This is a sample text for token counting.')}")

    except ValueError as e:
        print(f"Configuration error: {e}")
        print("Please set your COHERE_API_KEY environment variable to test embedding functionality.")
    except Exception as e:
        print(f"Error during testing: {e}")