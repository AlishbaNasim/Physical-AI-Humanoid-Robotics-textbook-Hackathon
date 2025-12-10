"""
Content models for the RAG chatbot system
"""

from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class Content(BaseModel):
    """Model for book content"""
    content_id: str
    title: str
    content: str
    module: str
    chapter: int
    section: int
    word_count: int
    estimated_reading_time: int
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()


class ContentSearchResult(BaseModel):
    """Model for content search results"""
    content_id: str
    title: str
    url: str
    excerpt: str
    relevance_score: float
    module: str


class ContentIngestRequest(BaseModel):
    """Model for content ingestion requests"""
    content_id: str
    title: str
    content: str
    module: str
    metadata: dict = {}