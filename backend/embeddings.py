import os
from typing import List
from sentence_transformers import SentenceTransformer

# Load the sentence transformer model
model = SentenceTransformer('all-MiniLM-L6-v2')

def generate_embedding(text: str) -> List[float]:
    """
    Generate an embedding for the given text using a local sentence transformer model.
    """
    try:
        # Use local model instead of OpenAI
        embedding = model.encode(text)
        return embedding.tolist()
    except Exception as e:
        raise Exception(f"Error generating embedding: {str(e)}")

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks of specified size.
    """
    if not text:
        return []

    # Split text into words
    words = text.split()

    # If text is already smaller than chunk size, return as is
    if len(words) <= chunk_size:
        return [text]

    chunks = []
    start_idx = 0

    while start_idx < len(words):
        # Get the end index for the current chunk
        end_idx = start_idx + chunk_size

        # Create the chunk
        chunk = " ".join(words[start_idx:end_idx])
        chunks.append(chunk)

        # Move the start index forward by chunk_size - overlap
        start_idx = end_idx - overlap

        # If the remaining text is smaller than chunk_size, add it as the final chunk
        if start_idx + chunk_size > len(words):
            if start_idx < len(words):
                final_chunk = " ".join(words[start_idx:])
                chunks.append(final_chunk)
            break

    return chunks

# Async version for use with async functions
async def generate_embedding_async(text: str) -> List[float]:
    """
    Async version of generate_embedding function.
    """
    return generate_embedding(text)