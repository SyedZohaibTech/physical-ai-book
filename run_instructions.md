# RAG Chatbot Setup and Run Instructions

## Backend Setup (FastAPI, OpenAI, Neon Postgres, Qdrant)

1.  **Navigate to the backend directory:**
    ```bash
    cd backend/
    ```

2.  **Create a `.env` file:**
    Copy the contents of `.env.example` to a new file named `.env` in the `backend/` directory.
    ```bash
    cp .env.example .env
    ```
    Then, open `.env` and fill in your actual credentials:
    ```
    OPENAI_API_KEY="your_openai_api_key_here"
    DATABASE_URL="postgresql+psycopg://user:password@host:port/dbname"
    # Example Neon connection string: postgresql+psycopg://user:password@ep-random-string-55555.us-east-2.aws.neon.tech/dbname
    QDRANT_URL="your_qdrant_url_here"
    QDRANT_API_KEY="your_qdrant_api_key_here"
    QDRANT_COLLECTION_NAME="physical_ai_textbook"
    ```

3.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

4.  **Start the FastAPI server:**
    ```bash
    uvicorn main:app --reload
    ```
    The backend should now be running on `http://localhost:8000`.

## Data Ingestion for Qdrant (Crucial for RAG functionality)

The current implementation has the Qdrant service set up, but the book content itself needs to be extracted, chunked, embedded, and then uploaded to Qdrant.

**Action Required:** You need to create a script (e.g., `ingest_data.py` in the `backend/` directory) that performs the following steps:

1.  **Read Book Content:** Parse the Markdown files (`.md` or `.mdx`) located in your `website/docs/` directory.
2.  **Chunk Text:** Break down the content into smaller, meaningful chunks (e.g., by paragraph or section).
3.  **Generate Embeddings:** For each chunk, use the `get_text_embedding` function from `backend/qdrant_service.py` to generate an embedding.
4.  **Upsert to Qdrant:** Use the `upsert_vectors_to_qdrant` function from `backend/qdrant_service.py` to upload the chunks and their embeddings (along with any useful metadata like source file or section) to your Qdrant collection.

**Example `ingest_data.py` (conceptual - you will need to expand this):**

```python
import os
from dotenv import load_dotenv
from qdrant_client.http.models import PointStruct, VectorParams, Distance

from qdrant_client import QdrantClient
from qdrant_service import get_text_embedding, create_qdrant_collection, COLLECTION_NAME, VECTOR_SIZE
import glob

# Load environment variables
load_dotenv()

def get_book_content_chunks(docs_path: str):
    """
    Reads markdown files from the docs_path, chunks them, and returns
    a list of (chunk_text, metadata) tuples.
    This is a simplified example; real chunking might be more complex.
    """
    chunks = []
    markdown_files = glob.glob(os.path.join(docs_path, '**/*.mdx'), recursive=True) + \
                     glob.glob(os.path.join(docs_path, '**/*.md'), recursive=True)
    
    for file_path in markdown_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            # Simple chunking: split by paragraphs or a fixed number of lines
            # You might want to use a more sophisticated text splitter here
            lines = content.split('\n')
            current_chunk = []
            for line in lines:
                current_chunk.append(line)
                if len(current_chunk) >= 5: # Chunk every 5 lines as an example
                    chunks.append((" ".join(current_chunk), {"source": os.path.basename(file_path)}))
                    current_chunk = []
            if current_chunk: # Add any remaining lines
                chunks.append((" ".join(current_chunk), {"source": os.path.basename(file_path)}))
    return chunks

if __name__ == "__main__":
    create_qdrant_collection() # Ensure collection exists

    docs_path = "../website/docs" # Path to your Docusaurus docs
    book_chunks_with_metadata = get_book_content_chunks(docs_path)

    texts_to_embed = [chunk[0] for chunk in book_chunks_with_metadata]
    metadatas = [chunk[1] for chunk in book_chunks_with_metadata]

    print(f"Ingesting {len(texts_to_embed)} chunks into Qdrant...")
    upsert_vectors_to_qdrant(texts_to_embed, metadatas)
    print("Ingestion complete.")
```

Run this script from the `backend/` directory:
```bash
python ingest_data.py
```

## Frontend Setup (React Chatbot in Docusaurus)

1.  **Navigate to the frontend directory:**
    ```bash
    cd website/
    ```

2.  **Create a `.env.development` file:**
    If not already present, create this file in the `website/` directory.
    ```
    REACT_APP_BACKEND_URL="http://localhost:8000"
    ```
    (Ensure this matches the address where your FastAPI backend is running).

3.  **Install Node.js dependencies:**
    ```bash
    npm install
    ```

4.  **Start the Docusaurus development server:**
    ```bash
    npm start
    ```
    Your Docusaurus site will open in your browser, and the chatbot should be visible on the `Introduction` page (`/docs/intro`).

This completes the setup and deployment instructions.
