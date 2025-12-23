"""
Qdrant database utilities for the RAG chatbot system.
Handles vector database operations using Qdrant.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from src.config import config
import uuid


class QdrantDB:
    """
    Qdrant database client for vector storage and retrieval.
    """

    def __init__(self):
        """
        Initialize the Qdrant client with configuration.
        """
        self.client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            prefer_grpc=False  # Using REST API
        )
        self.collection_name = config.qdrant_collection_name

    def create_collection(self) -> bool:
        """
        Create the Qdrant collection with proper schema for book content chunks.

        Returns:
            bool: True if collection was created or already exists
        """
        try:
            # Check if collection already exists
            try:
                collection_info = self.client.get_collection(collection_name=self.collection_name)
                # Collection exists, return True
                print(f"Collection '{self.collection_name}' already exists")
                return True
            except:
                # Collection doesn't exist, proceed to create it
                pass

            # Create collection with 1024-dimensional vectors (for Cohere embed-english-v3.0)
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1024,  # Cohere embed-english-v3.0 produces 1024-dimensional vectors
                    distance=models.Distance.COSINE
                )
            )

            # Create payload index for efficient filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="book_id",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            print(f"Collection '{self.collection_name}' created successfully")
            return True

        except Exception as e:
            print(f"Error creating collection: {e}")
            return False

    def insert_chunks(self, chunks: List[Dict[str, Any]]) -> bool:
        """
        Insert book content chunks into the Qdrant collection.

        Args:
            chunks: List of chunk dictionaries with text_content, metadata, and embeddings

        Returns:
            bool: True if insertion was successful
        """
        if not chunks:
            print("No chunks to insert")
            return True

        try:
            points = []
            for chunk in chunks:
                # Generate a unique ID for each chunk if not provided
                chunk_id = chunk.get("chunk_id", str(uuid.uuid4()))
                # Store the chunk_id back to the chunk for verification purposes
                chunk["chunk_id"] = chunk_id

                # Extract required fields
                text_content = chunk.get("text_content", "")
                embedding = chunk.get("embedding")
                metadata = chunk.get("metadata", {})

                # Validate required fields
                if not text_content or not embedding:
                    print(f"Skipping chunk due to missing required fields: text_content={bool(text_content)}, embedding={bool(embedding)}")
                    continue

                # Add book_id to metadata if not present
                if "book_id" not in metadata:
                    metadata["book_id"] = "physical-ai"

                # Create point structure
                point = PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        "chunk_text": text_content,
                        "chunk_id": chunk_id,
                        "url": metadata.get("url", ""),
                        "section": metadata.get("section", ""),
                        "book_id": metadata.get("book_id", "physical-ai"),
                        "metadata": metadata
                    }
                )

                points.append(point)

            if not points:
                print("No valid points to insert after validation")
                return False

            # Insert points into collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"Successfully inserted {len(points)} chunks into collection '{self.collection_name}'")
            return True

        except Exception as e:
            print(f"Error inserting chunks: {e}")
            return False

    def search_chunks(self, query_embedding: List[float], limit: int = 8, book_id: str = "physical-ai") -> List[Dict[str, Any]]:
        """
        Search for relevant chunks based on query embedding.

        Args:
            query_embedding: Embedding vector for the query
            limit: Maximum number of results to return
            book_id: ID of the book to search within

        Returns:
            List of matching chunks with similarity scores
        """
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_id",
                            match=models.MatchValue(value=book_id)
                        )
                    ]
                ),
                limit=limit,
                with_payload=True,
                with_vectors=False
            ).points

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "chunk_id": result.id,
                    "text_content": result.payload.get("chunk_text", ""),
                    "url": result.payload.get("url", ""),
                    "section": result.payload.get("section", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "similarity_score": result.score
                })

            return formatted_results

        except Exception as e:
            print(f"Error searching chunks: {e}")
            return []

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution!).

        Returns:
            bool: True if deletion was successful
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' deleted successfully")
            return True
        except Exception as e:
            print(f"Error deleting collection: {e}")
            return False

    def count_points(self) -> int:
        """
        Count the number of points in the collection.

        Returns:
            int: Number of points in the collection
        """
        try:
            count = self.client.count(collection_name=self.collection_name)
            return count.count
        except Exception as e:
            print(f"Error counting points: {e}")
            return 0

    def verify_insertion(self, chunk_ids: List[str]) -> bool:
        """
        Verify that specific chunks were properly inserted by checking for their existence.

        Args:
            chunk_ids: List of chunk IDs to verify

        Returns:
            bool: True if all chunks exist in the collection
        """
        try:
            if not chunk_ids:
                return True

            # Get points by IDs - use scroll method instead of query_points with ids
            points = []
            for chunk_id in chunk_ids:
                result = self.client.retrieve(
                    collection_name=self.collection_name,
                    ids=[chunk_id],
                    with_payload=True,
                    with_vectors=False
                )
                points.extend(result)

            # Check if all requested IDs were found
            found_ids = [point.id for point in points]
            return all(chunk_id in found_ids for chunk_id in chunk_ids)

        except Exception as e:
            print(f"Error verifying insertion: {e}")
            return False


# Example usage
if __name__ == "__main__":
    db = QdrantDB()

    # Create collection
    db.create_collection()

    # Example chunk (would normally have a real embedding)
    sample_chunk = {
        "chunk_id": "test-chunk-1",
        "text_content": "This is a sample book content chunk for testing.",
        "embedding": [0.1] * 1024,  # Example embedding vector
        "metadata": {
            "url": "http://example.com/chapter1",
            "section": "Introduction",
            "book_id": "physical-ai"
        }
    }

    # Insert sample chunk
    success = db.insert_chunks([sample_chunk])
    print(f"Insert successful: {success}")

    # Count points
    count = db.count_points()
    print(f"Total points in collection: {count}")