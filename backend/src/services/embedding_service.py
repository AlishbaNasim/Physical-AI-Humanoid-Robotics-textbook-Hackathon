from openai import AsyncOpenAI
from typing import List
import os
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    openai_model: str = os.getenv("OPENAI_MODEL", "text-embedding-ada-002")

    class Config:
        env_file = ".env"

settings = Settings()

class EmbeddingService:
    def __init__(self):
        if settings.openai_api_key:
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        else:
            self.client = None  # Placeholder for when API key is not available

    async def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for the provided texts using OpenAI"""
        if not self.client:
            # Placeholder implementation when API key is not available
            return [[0.1, 0.2, 0.3] for _ in texts]  # Dummy embeddings

        # In a real implementation, you would call the OpenAI API
        # This is just a placeholder to avoid actual API calls during setup
        return [[0.1, 0.2, 0.3] for _ in texts]  # Dummy embeddings

    async def create_single_embedding(self, text: str) -> List[float]:
        """Create a single embedding for the provided text"""
        embeddings = await self.create_embeddings([text])
        return embeddings[0] if embeddings else [0.0]