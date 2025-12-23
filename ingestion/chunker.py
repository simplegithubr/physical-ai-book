"""
Text chunking module for the RAG chatbot system.
Implements text chunking logic with 600-800 token chunks and 200-token overlap.
"""
import re
from typing import List, Dict, Any


class TextChunker:
    """
    Handles text chunking with boundary awareness and overlap.
    """

    def __init__(self, max_chunk_size: int = 700, overlap_size: int = 200):
        """
        Initialize the chunker with specified parameters.

        Args:
            max_chunk_size: Maximum size of each chunk in tokens
            overlap_size: Number of tokens to overlap between chunks
        """
        self.max_chunk_size = max_chunk_size
        self.overlap_size = overlap_size

    def chunk_text(self, text: str, metadata: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Split text into chunks with overlap, respecting section boundaries.

        Args:
            text: The text to chunk
            metadata: Additional metadata to include with each chunk

        Returns:
            List of chunk dictionaries with text and metadata
        """
        if not text:
            return []

        # Split text into sentences to respect boundaries
        sentences = self._split_into_sentences(text)
        chunks = []
        current_chunk = ""
        current_size = 0

        for i, sentence in enumerate(sentences):
            sentence_size = len(sentence.split())  # Simple token approximation

            # If adding this sentence would exceed max size
            if current_size + sentence_size > self.max_chunk_size and current_chunk:
                # Save current chunk
                chunk_id = f"chunk_{len(chunks) + 1}"
                chunk = {
                    "chunk_id": chunk_id,
                    "text_content": current_chunk.strip(),
                    "metadata": metadata or {},
                    "size": current_size
                }
                chunks.append(chunk)

                # Start new chunk with overlap if possible
                if self.overlap_size > 0:
                    # Find overlapping text from the end of current chunk
                    overlap_sentences = self._get_overlap_sentences(current_chunk, self.overlap_size)
                    current_chunk = overlap_sentences + " " + sentence
                    current_size = len(current_chunk.split())
                else:
                    current_chunk = sentence
                    current_size = sentence_size
            else:
                current_chunk += " " + sentence
                current_size += sentence_size

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunk_id = f"chunk_{len(chunks) + 1}"
            chunk = {
                "chunk_id": chunk_id,
                "text_content": current_chunk.strip(),
                "metadata": metadata or {},
                "size": current_size
            }
            chunks.append(chunk)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using regex.

        Args:
            text: Input text to split

        Returns:
            List of sentences
        """
        # Split on sentence endings followed by whitespace and capital letter
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def _get_overlap_sentences(self, text: str, token_count: int) -> str:
        """
        Get the last N tokens from text to use as overlap.

        Args:
            text: Input text
            token_count: Number of tokens to get from the end

        Returns:
            Overlapping text
        """
        tokens = text.split()
        if len(tokens) <= token_count:
            return text

        overlap_tokens = tokens[-token_count:]
        return " ".join(overlap_tokens)


# Example usage
if __name__ == "__main__":
    chunker = TextChunker(max_chunk_size=50, overlap_size=10)
    sample_text = """
    This is the first sentence. This is the second sentence. The third sentence follows here.
    The fourth sentence continues the text. Finally, the fifth sentence concludes this paragraph.
    This is a new paragraph with its own sentences. More content in the second sentence.
    The third sentence provides additional information. The fourth sentence wraps up this paragraph.
    """

    chunks = chunker.chunk_text(sample_text, {"source": "test", "url": "http://example.com"})

    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1}: {chunk['size']} tokens")
        print(f"Content: {chunk['text_content'][:100]}...")
        print("---")