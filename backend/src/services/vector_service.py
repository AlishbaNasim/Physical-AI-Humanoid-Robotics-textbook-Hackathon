from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import os
from pydantic_settings import BaseSettings
import logging

class Settings(BaseSettings):
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")

    class Config:
        env_file = ".env"

settings = Settings()

class VectorService:
    def __init__(self):
        # Initialize Qdrant client with actual connection
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        else:
            # For local development
            self.client = QdrantClient(host="localhost", port=6333)

        self.collection_name = "book_content"
        # Initialize collection if it doesn't exist
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the Qdrant collection for book content"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Default OpenAI embedding size
                        distance=models.Distance.COSINE
                    )
                )
                logging.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logging.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logging.error(f"Error initializing Qdrant collection: {e}")

    async def search_similar(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar content in the vector database"""
        try:
            # In a real implementation, you would:
            # 1. Generate embeddings for the query using embedding_service
            # 2. Search the Qdrant collection
            # 3. Return similar content with relevance scores

            # This is a placeholder that simulates the search
            # In real implementation, you'd call embedding_service.create_single_embedding(query)
            # and then use self.client.search to find similar vectors
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=[0.1] * 1536,  # Placeholder vector - in real implementation, use actual embedding
                limit=limit,
                with_payload=True
            )

            # Format results to match expected structure
            formatted_results = []
            for result in results:
                payload = result.payload
                formatted_results.append({
                    "content_id": payload.get("content_id", ""),
                    "title": payload.get("title", ""),
                    "url": payload.get("url", ""),
                    "text": payload.get("text", ""),
                    "relevance_score": float(result.score)
                })

            return formatted_results
        except Exception as e:
            logging.error(f"Error searching similar content: {e}")
            # Return fallback results in case of error
            return [
                {
                    "content_id": "fallback-content",
                    "title": "Fallback Content",
                    "url": "/docs/intro",
                    "text": "This is fallback content in case of search error.",
                    "relevance_score": 0.5
                }
            ]

    async def add_content(self, content_id: str, title: str, content: str, metadata: Dict[str, Any] = None):
        """Add content to the vector database"""
        try:
            # In a real implementation, you would:
            # 1. Generate embeddings for the content
            # 2. Store in Qdrant with the content as payload
            # 3. Return the vector ID

            # For now, we'll simulate adding content by generating a vector ID
            # In real implementation, you'd call embedding_service.create_single_embedding(content)
            vector_id = f"vec_{content_id}"

            # Add to Qdrant (placeholder implementation)
            # In real implementation:
            # embedding = await embedding_service.create_single_embedding(content)
            # self.client.upsert(
            #     collection_name=self.collection_name,
            #     points=[
            #         models.PointStruct(
            #             id=vector_id,
            #             vector=embedding,
            #             payload={
            #                 "content_id": content_id,
            #                 "title": title,
            #                 "text": content,
            #                 "url": metadata.get("url", "") if metadata else "",
            #                 **(metadata or {})
            #             }
            #         )
            #     ]
            # )

            return vector_id
        except Exception as e:
            logging.error(f"Error adding content to vector database: {e}")
            raise e

    async def delete_content(self, content_id: str):
        """Remove content from the vector database"""
        try:
            # In a real implementation, you would:
            # 1. Find the vector ID associated with content_id
            # 2. Delete from Qdrant
            # 3. Return success status

            # Placeholder implementation - in real implementation:
            # self.client.delete(
            #     collection_name=self.collection_name,
            #     points_selector=models.Filter(
            #         must=[
            #             models.FieldCondition(
            #                 key="content_id",
            #                 match=models.MatchValue(value=content_id)
            #             )
            #         ]
            #     )
            # )

            return True
        except Exception as e:
            logging.error(f"Error deleting content from vector database: {e}")
            raise e