"""
BookIngestionService for the RAG chatbot system.
Handles book content ingestion, processing, and indexing.
"""
from typing import List, Dict, Any, Optional
from src.database.qdrant import QdrantDB
from src.database.neon import NeonDB
from src.services.embedding_service import EmbeddingService
from src.models.book_chunk import BookChunk
from src.utils.errors import DatabaseConnectionError
from src.utils.logging import get_logger
import asyncio
import requests
from bs4 import BeautifulSoup
import time
import uuid


logger = get_logger(__name__)


class BookIngestionService:
    """
    Service class for handling book content ingestion, processing, and indexing.
    """

    def __init__(self):
        """
        Initialize the BookIngestionService with required dependencies.
        """
        self.qdrant_db = QdrantDB()
        self.neon_db = NeonDB()
        self.embedding_service = EmbeddingService()

    async def ingest_book_from_url(self, source_url: str, force_reindex: bool = False) -> Dict[str, Any]:
        """
        Ingest book content from a URL source.

        Args:
            source_url: URL of the book site to scrape and index
            force_reindex: Whether to force reindexing even if content exists

        Returns:
            Dict with ingestion status and statistics
        """
        start_time = time.time()

        try:
            logger.info(f"Starting ingestion from URL: {source_url}")

            # Create Qdrant collection if it doesn't exist
            collection_created = self.qdrant_db.create_collection()
            if not collection_created:
                raise DatabaseConnectionError("Failed to create Qdrant collection")

            # Scrape content from the URL
            book_content = await self._scrape_content_from_url(source_url)

            if not book_content:
                raise ValueError(f"No content found at URL: {source_url}")

            # Process and chunk the content
            chunks = self._process_content(book_content, source_url)

            # Generate embeddings for the chunks
            chunk_embeddings = await self._generate_embeddings(chunks)

            # Insert chunks into Qdrant
            success = self.qdrant_db.insert_chunks(chunk_embeddings)
            if not success:
                raise DatabaseConnectionError("Failed to insert chunks into Qdrant")

            # Verify that chunks were properly inserted
            inserted_chunk_ids = [chunk.get("chunk_id") for chunk in chunk_embeddings if chunk.get("chunk_id")]
            if inserted_chunk_ids:
                verification_success = self.qdrant_db.verify_insertion(inserted_chunk_ids)
                if not verification_success:
                    logger.warning(f"Verification failed for some chunks: {len(inserted_chunk_ids)} chunks inserted but not all verified")
                else:
                    logger.info(f"Successfully verified {len(inserted_chunk_ids)} chunks in Qdrant")

            # Update metadata in Neon Postgres
            await self._update_metadata(source_url, len(chunk_embeddings))

            # Calculate statistics
            duration = time.time() - start_time

            result = {
                "status": "completed",
                "source_url": source_url,
                "chunks_processed": len(chunk_embeddings),
                "duration_seconds": round(duration, 2),
                "force_reindex": force_reindex
            }

            logger.info(f"Ingestion completed: {result}")

            return result

        except Exception as e:
            logger.error(f"Error during ingestion: {str(e)}", exc_info=True)
            raise

    async def _scrape_content_from_url(self, url: str) -> str:
        """
        Scrape content from a URL.

        Args:
            url: URL to scrape content from

        Returns:
            Scraped content as a string
        """
        try:
            logger.info(f"Scraping content from: {url}")

            # Make request to get the page content
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            response = requests.get(url, headers=headers)
            response.raise_for_status()

            # Parse HTML content
            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text content
            text = soup.get_text()

            # Clean up text (remove extra whitespace)
            lines = (line.strip() for line in text.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text = ' '.join(chunk for chunk in chunks if chunk)

            logger.info(f"Scraped content length: {len(text)} characters")
            return text

        except requests.RequestException as e:
            logger.error(f"Error scraping URL {url}: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Error parsing content from {url}: {str(e)}")
            raise

    def _process_content(self, content: str, source_url: str) -> List[Dict[str, Any]]:
        """
        Process and chunk the book content using simple text splitting.

        Args:
            content: Raw book content to process
            source_url: Source URL for metadata

        Returns:
            List of processed content chunks
        """
        try:
            logger.info(f"Processing content with simple chunker (source: {source_url})")

            # Simple chunking: split content into chunks of approximately 1000 characters
            chunk_size = 1000
            chunks = []

            # Split content into chunks
            for i in range(0, len(content), chunk_size):
                chunk_text = content[i:i + chunk_size]

                # Create chunk with metadata
                chunk = {
                    "chunk_id": str(uuid.uuid4()),  # Use UUID as Qdrant expects
                    "text_content": chunk_text,
                    "metadata": {
                        "url": source_url,
                        "source_type": "web_scraped",
                        "processed_at": time.time(),
                        "chunk_index": i // chunk_size,
                        "total_chunks": (len(content) - 1) // chunk_size + 1
                    }
                }
                chunks.append(chunk)

            logger.info(f"Content processed into {len(chunks)} chunks")
            return chunks

        except Exception as e:
            logger.error(f"Error processing content: {str(e)}", exc_info=True)
            raise

    async def _generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for the content chunks using the embedding service.

        Args:
            chunks: List of content chunks

        Returns:
            List of chunks with embeddings added
        """
        try:
            logger.info(f"Generating embeddings for {len(chunks)} chunks")

            # Extract text content for embedding
            texts = [chunk["text_content"] for chunk in chunks]

            # Generate embeddings using the embedding service
            embeddings = self.embedding_service.embed_texts(texts)

            # Add embeddings to chunks
            chunks_with_embeddings = []
            for i, chunk in enumerate(chunks):
                chunk_with_embedding = chunk.copy()
                chunk_with_embedding["embedding"] = embeddings[i]
                chunks_with_embeddings.append(chunk_with_embedding)

            logger.info(f"Generated embeddings for {len(chunks_with_embeddings)} chunks")
            return chunks_with_embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}", exc_info=True)
            raise

    async def _update_metadata(self, source_url: str, total_chunks: int) -> None:
        """
        Update book metadata in the database.

        Args:
            source_url: Source URL of the ingested content
            total_chunks: Total number of chunks processed
        """
        try:
            # Connect to Neon database
            await self.neon_db.connect()
            await self.neon_db.create_tables()

            # Get current timestamp for version
            import datetime
            version = datetime.datetime.utcnow().strftime("%Y.%m.%d.%H%M%S")

            # Upsert book metadata
            success = await self.neon_db.upsert_book_metadata(
                book_id="physical-ai",
                version=version,
                total_chunks=total_chunks
            )

            if success:
                logger.info(f"Metadata updated: {total_chunks} chunks, version {version}")
            else:
                logger.warning("Failed to update book metadata")

        except Exception as e:
            logger.error(f"Error updating metadata: {str(e)}", exc_info=True)
            raise

    async def ingest_from_text(self, text: str, source_metadata: Dict[str, Any]) -> Dict[str, Any]:
        """
        Ingest book content from raw text.

        Args:
            text: Raw book content as text
            source_metadata: Metadata about the source

        Returns:
            Dict with ingestion status and statistics
        """
        start_time = time.time()

        try:
            logger.info(f"Starting ingestion from text (length: {len(text)} chars)")

            # Create Qdrant collection if it doesn't exist
            collection_created = self.qdrant_db.create_collection()
            if not collection_created:
                raise DatabaseConnectionError("Failed to create Qdrant collection")

            # Process and chunk the content
            chunks = self._process_content(text, source_metadata.get("url", "unknown"))

            # Generate embeddings for the chunks
            chunk_embeddings = await self._generate_embeddings(chunks)

            # Insert chunks into Qdrant
            success = self.qdrant_db.insert_chunks(chunk_embeddings)
            if not success:
                raise DatabaseConnectionError("Failed to insert chunks into Qdrant")

            # Verify that chunks were properly inserted
            inserted_chunk_ids = [chunk.get("chunk_id") for chunk in chunk_embeddings if chunk.get("chunk_id")]
            if inserted_chunk_ids:
                verification_success = self.qdrant_db.verify_insertion(inserted_chunk_ids)
                if not verification_success:
                    logger.warning(f"Verification failed for some chunks: {len(inserted_chunk_ids)} chunks inserted but not all verified")
                else:
                    logger.info(f"Successfully verified {len(inserted_chunk_ids)} chunks in Qdrant")

            # Update metadata in Neon Postgres
            await self._update_metadata(source_metadata.get("url", "unknown"), len(chunk_embeddings))

            # Calculate statistics
            duration = time.time() - start_time

            result = {
                "status": "completed",
                "source_type": "text",
                "chunks_processed": len(chunk_embeddings),
                "duration_seconds": round(duration, 2)
            }

            logger.info(f"Text ingestion completed: {result}")

            return result

        except Exception as e:
            logger.error(f"Error during text ingestion: {str(e)}", exc_info=True)
            raise

    def get_ingestion_status(self) -> Dict[str, Any]:
        """
        Get the current status of the ingestion service.

        Returns:
            Dict with ingestion service status
        """
        try:
            # Check Qdrant connection
            qdrant_status = self.qdrant_db.count_points() >= 0

            # Check if collection exists
            try:
                self.qdrant_db.client.get_collection(self.qdrant_db.collection_name)
                collection_exists = True
            except:
                collection_exists = False

            status = {
                "qdrant_connected": qdrant_status,
                "collection_exists": collection_exists,
                "qdrant_points_count": self.qdrant_db.count_points() if qdrant_status else 0,
                "service_ready": qdrant_status and collection_exists
            }

            return status

        except Exception as e:
            logger.error(f"Error getting ingestion status: {str(e)}", exc_info=True)
            return {
                "qdrant_connected": False,
                "collection_exists": False,
                "qdrant_points_count": 0,
                "service_ready": False,
                "error": str(e)
            }


# Example usage
if __name__ == "__main__":
    import asyncio

    async def example_usage():
        service = BookIngestionService()

        # Check service status
        status = service.get_ingestion_status()
        print(f"Ingestion service status: {status}")

        # Example of ingesting from a text
        sample_text = """
        Chapter 1: Introduction to Physical AI

        Physical AI is a revolutionary field that combines robotics with artificial intelligence to create systems that can interact with the physical world intelligently. This approach differs from traditional AI by emphasizing the importance of physical embodiment in creating truly intelligent systems.

        The core principles of Physical AI include:
        1. Embodied intelligence - Intelligence emerges from the interaction between an agent and its environment
        2. Sensorimotor learning - Learning through physical interaction and feedback
        3. Adaptive behavior - Systems that can adapt their behavior based on physical experiences

        Chapter 2: Key Technologies

        The technologies that enable Physical AI include advanced robotics, machine learning, computer vision, and sensor fusion. These technologies work together to create systems that can perceive, reason, and act in physical spaces.
        """

        try:
            result = await service.ingest_from_text(
                text=sample_text,
                source_metadata={"url": "test://sample-book", "title": "Sample Book"}
            )
            print(f"Ingestion result: {result}")
        except Exception as e:
            print(f"Error during ingestion: {e}")

    # Note: This example won't run without proper API keys configured
    # asyncio.run(example_usage())