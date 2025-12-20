import os
import sys

# Add the backend directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'backend')))

from embeddings import setup_qdrant_collection, index_book_content

if __name__ == "__main__":
    COLLECTION_NAME = "physical-ai-book"
    DOCS_DIRECTORY = "docs"

    setup_qdrant_collection(collection_name=COLLECTION_NAME)
    index_book_content(directory_path=DOCS_DIRECTORY, collection_name=COLLECTION_NAME)
