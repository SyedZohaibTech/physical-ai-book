import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class VectorStore:
    def __init__(self):
        # Initialize Qdrant client with environment variables
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

        if not self.qdrant_api_key:
            self.client = QdrantClient(url=self.qdrant_url)
        else:
            self.client = QdrantClient(url=self.qdrant_url, api_key=self.qdrant_api_key)

        self.collection_name = "physical_ai_book"
        self.vector_size = 384  # Size for sentence-transformers all-MiniLM-L6-v2

        # Create collection if it doesn't exist
        self.create_collection()

    def create_collection(self) -> bool:
        """
        Create the collection for storing textbook content embeddings.
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name not in collection_names:
                # Create the collection
                self.client.recreate_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
                )
                logger.info(f"Created collection '{self.collection_name}' with {self.vector_size}-dimension vectors")
                return True
            else:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True
        except Exception as e:
            logger.error(f"Error creating collection: {str(e)}")
            return False

    def store_embedding(self, content: str, embedding: List[float], metadata: Dict[str, Any]) -> bool:
        """
        Store an embedding with its content and metadata in the vector store.
        """
        try:
            # Prepare the point for insertion
            points = [
                models.PointStruct(
                    id=hash(content) % (10**9),  # Simple hash-based ID generation
                    vector=embedding,
                    payload={
                        "content": content,
                        "file_path": metadata.get("file_path", ""),
                        "module": metadata.get("module", ""),
                        "title": metadata.get("title", ""),
                        "chunk_index": metadata.get("chunk_index", 0),
                    }
                )
            ]

            # Insert the point into the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Stored embedding for content: {content[:50]}...")
            return True
        except Exception as e:
            logger.error(f"Error storing embedding: {str(e)}")
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar content based on the query embedding.
        Returns top matches with score > 0.7.
        """
        try:
            # Search for similar vectors in the collection
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=0.7  # Only return results with score > 0.7
            )

            # Format results
            results = []
            for hit in search_results:
                if hit.score > 0.7:  # Double-check the score threshold
                    results.append({
                        "id": hit.id,
                        "score": hit.score,
                        "content": hit.payload.get("content", ""),
                        "file_path": hit.payload.get("file_path", ""),
                        "module": hit.payload.get("module", ""),
                        "title": hit.payload.get("title", ""),
                        "chunk_index": hit.payload.get("chunk_index", 0),
                    })

            logger.info(f"Found {len(results)} similar results with score > 0.7")
            return results
        except Exception as e:
            logger.error(f"Error searching for similar content: {str(e)}")
            return []

    def health_check(self) -> bool:
        """
        Check if the Qdrant connection is healthy.
        """
        try:
            # Try to get collections to verify connection
            self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            return False