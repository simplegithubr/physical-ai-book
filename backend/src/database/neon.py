"""
Neon Postgres database utilities for the RAG chatbot system.
Handles relational database operations using Neon Serverless Postgres.
"""
from typing import Optional, Dict, Any, List
import os
from datetime import datetime
from src.config import config

try:
    import asyncpg
    ASYNCPG_AVAILABLE = True
except ImportError:
    ASYNCPG_AVAILABLE = False


class NeonDB:
    """
    Neon Postgres database client for metadata storage.
    """

    def __init__(self):
        """
        Initialize the Neon database connection pool.
        """
        self.connection_string = config.neon_database_url
        self.pool = None

    async def connect(self):
        """
        Create a connection pool to the Neon database.
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping Neon database connection")
            return

        try:
            self.pool = await asyncpg.create_pool(
                dsn=self.connection_string,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            print("Connected to Neon database successfully")
        except Exception as e:
            print(f"Error connecting to Neon database: {e}")
            raise

    async def disconnect(self):
        """
        Close the connection pool.
        """
        if ASYNCPG_AVAILABLE and self.pool:
            await self.pool.close()
            print("Disconnected from Neon database")

    async def create_tables(self):
        """
        Create the required tables if they don't exist.
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping table creation")
            return True

        if not self.pool:
            await self.connect()

        create_table_query = """
        CREATE TABLE IF NOT EXISTS book_metadata (
            id SERIAL PRIMARY KEY,
            book_id VARCHAR(255) UNIQUE NOT NULL,
            version VARCHAR(50),
            total_chunks INTEGER DEFAULT 0,
            last_ingested TIMESTAMP WITH TIME ZONE,
            total_pages INTEGER DEFAULT 0,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
        );

        -- Create indexes if they don't exist
        CREATE INDEX IF NOT EXISTS idx_book_metadata_book_id ON book_metadata(book_id);
        CREATE INDEX IF NOT EXISTS idx_book_metadata_last_ingested ON book_metadata(last_ingested);
        """

        try:
            async with self.pool.acquire() as connection:
                await connection.execute(create_table_query)
            print("Tables created successfully (if not existed)")
            return True
        except Exception as e:
            print(f"Error creating tables: {e}")
            raise

    async def upsert_book_metadata(self, book_id: str, version: Optional[str] = None,
                                   total_chunks: Optional[int] = None,
                                   total_pages: Optional[int] = None) -> bool:
        """
        Insert or update book metadata in the database.

        Args:
            book_id: Unique identifier for the book
            version: Version of the book
            total_chunks: Total number of content chunks
            total_pages: Total number of pages in the book

        Returns:
            bool: True if operation was successful
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping metadata upsert")
            return False

        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as connection:
                # Check if record exists
                existing = await connection.fetchrow(
                    "SELECT * FROM book_metadata WHERE book_id = $1", book_id
                )

                if existing:
                    # Update existing record
                    update_query = """
                    UPDATE book_metadata
                    SET total_chunks = COALESCE($2, total_chunks),
                        total_pages = COALESCE($3, total_pages),
                        last_ingested = NOW(),
                        version = COALESCE($4, version),
                        updated_at = NOW()
                    WHERE book_id = $1
                    """
                    await connection.execute(update_query, book_id, total_chunks, total_pages, version)
                else:
                    # Insert new record
                    insert_query = """
                    INSERT INTO book_metadata (book_id, version, total_chunks, total_pages, last_ingested)
                    VALUES ($1, $2, $3, $4, NOW())
                    """
                    await connection.execute(insert_query, book_id, version, total_chunks, total_pages)

            return True
        except Exception as e:
            print(f"Error upserting book metadata: {e}")
            return False

    async def get_book_metadata(self, book_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve book metadata from the database.

        Args:
            book_id: Unique identifier for the book

        Returns:
            Dict with book metadata or None if not found
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping metadata retrieval")
            return None

        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as connection:
                result = await connection.fetchrow(
                    "SELECT * FROM book_metadata WHERE book_id = $1", book_id
                )

                if result:
                    return {
                        "id": result["id"],
                        "book_id": result["book_id"],
                        "version": result["version"],
                        "total_chunks": result["total_chunks"],
                        "last_ingested": result["last_ingested"],
                        "total_pages": result["total_pages"],
                        "created_at": result["created_at"],
                        "updated_at": result["updated_at"]
                    }
                return None
        except Exception as e:
            print(f"Error getting book metadata: {e}")
            return None

    async def get_all_books_metadata(self) -> List[Dict[str, Any]]:
        """
        Retrieve metadata for all books in the database.

        Returns:
            List of book metadata dictionaries
        """
        if not ASYNCPG_AVAILABLE:
            print("asyncpg not available, skipping metadata retrieval")
            return []

        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as connection:
                results = await connection.fetch("SELECT * FROM book_metadata ORDER BY created_at DESC")

                books = []
                for result in results:
                    books.append({
                        "id": result["id"],
                        "book_id": result["book_id"],
                        "version": result["version"],
                        "total_chunks": result["total_chunks"],
                        "last_ingested": result["last_ingested"],
                        "total_pages": result["total_pages"],
                        "created_at": result["created_at"],
                        "updated_at": result["updated_at"]
                    })

                return books
        except Exception as e:
            print(f"Error getting all books metadata: {e}")
            return []


# Example usage
async def example_usage():
    db = NeonDB()

    try:
        # Connect and create tables
        await db.connect()
        await db.create_tables()

        # Insert/update book metadata
        success = await db.upsert_book_metadata(
            book_id="physical-ai",
            version="1.0.0",
            total_chunks=150,
            total_pages=300
        )
        print(f"Upsert successful: {success}")

        # Retrieve book metadata
        metadata = await db.get_book_metadata("physical-ai")
        print(f"Retrieved metadata: {metadata}")

        # Get all books
        all_books = await db.get_all_books_metadata()
        print(f"All books: {all_books}")

    finally:
        await db.disconnect()


if __name__ == "__main__":
    import asyncio
    asyncio.run(example_usage())