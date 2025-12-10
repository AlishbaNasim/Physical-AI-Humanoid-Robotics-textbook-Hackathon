from pydantic import BaseModel
from typing import List, Optional
import datetime

class ChatMessage(BaseModel):
    id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime.datetime
    sources: Optional[List[str]] = []

class ChatSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime.datetime
    updated_at: datetime.datetime
    messages: List[ChatMessage] = []

class ContentSource(BaseModel):
    content_id: str
    title: str
    url: str
    text: str
    relevance_score: float
    page_reference: Optional[str] = None