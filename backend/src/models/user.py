"""
User models for the RAG chatbot system
"""

from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class User(BaseModel):
    """Model for users"""
    user_id: str
    username: Optional[str] = None
    email: Optional[str] = None
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    preferences: dict = {}