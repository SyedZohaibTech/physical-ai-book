import os
import re
from pathlib import Path
from embeddings import generate_embedding, chunk_text
from vector_store import VectorStore
import logging
from tqdm import tqdm
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def remove_frontmatter(content):
    """
    Remove YAML frontmatter from markdown content (lines between ---).
    """
    lines = content.split('\n')
    frontmatter_started = False
    frontmatter_ended = False
    cleaned_lines = []

    for line in lines:
        if line.strip() == '---' and not frontmatter_started:
            frontmatter_started = True
            continue
        elif line.strip() == '---' and frontmatter_started and not frontmatter_ended:
            frontmatter_ended = True
            continue

        if not frontmatter_started or frontmatter_ended:
            cleaned_lines.append(line)

    return '\n'.join(cleaned_lines)

def extract_metadata(content):
    """
    Extract metadata from YAML frontmatter.
    """
    metadata = {}
    lines = content.split('\n')
    frontmatter_started = False
    frontmatter_ended = False
    frontmatter_lines = []

    for line in lines:
        if line.strip() == '---' and not frontmatter_started:
            frontmatter_started = True
            continue
        elif line.strip() == '---' and frontmatter_started and not frontmatter_ended:
            frontmatter_ended = True
            continue

        if frontmatter_started and not frontmatter_ended:
            frontmatter_lines.append(line)

    # Parse frontmatter if it exists
    if frontmatter_lines:
        for line in frontmatter_lines:
            if ':' in line:
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()

    return metadata

def embed_book_content():
    """
    Walk through docs/ folder recursively finding .md files,
    read each file, remove frontmatter, extract metadata,
    chunk content, generate embeddings, and store in Qdrant.
    """
    # Initialize vector store
    vector_store = VectorStore()

    # Find all markdown files in docs/ directory
    # Use relative path from backend/ to website/docs/
    docs_path = os.path.join(os.path.dirname(__file__), "..", "website", "docs")
    logger.info(f"Looking for docs in: {docs_path}")

    if not os.path.exists(docs_path):
        logger.error(f"docs directory not found at {docs_path}")
        return

    # Convert to Path object for rglob
    docs_path_obj = Path(docs_path)
    md_files = list(docs_path_obj.rglob("*.md"))
    logger.info(f"Found {len(md_files)} markdown files to process")

    total_files = len(md_files)
    total_chunks = 0

    # Process each file with a progress bar
    for file_path in tqdm(md_files, desc="Processing files", unit="file"):
        try:
            # Read the file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata from frontmatter
            metadata = extract_metadata(content)

            # Remove frontmatter from content
            cleaned_content = remove_frontmatter(content)

            # Chunk the content
            chunks = chunk_text(cleaned_content, chunk_size=500, overlap=50)

            # Process each chunk
            for i, chunk in enumerate(chunks):
                # Generate embedding for the chunk
                embedding = generate_embedding(chunk)

                # Prepare metadata for storage
                chunk_metadata = {
                    "file_path": str(file_path),
                    "module": metadata.get("module", ""),
                    "title": metadata.get("title", ""),
                    "chunk_index": i,
                }

                # Store the embedding in the vector store
                success = vector_store.store_embedding(
                    content=chunk,
                    embedding=embedding,
                    metadata=chunk_metadata
                )

                if not success:
                    logger.error(f"Failed to store embedding for chunk {i} of {file_path}")

                total_chunks += 1

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {str(e)}")
            continue

    logger.info(f"Embedding completed! Processed {total_files} files and {total_chunks} chunks")

if __name__ == "__main__":
    embed_book_content()