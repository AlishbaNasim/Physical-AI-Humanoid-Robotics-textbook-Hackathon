from sqlalchemy import Column, String, DateTime, Text, select
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.dialects.postgresql import UUID
from pydantic_settings import BaseSettings
from typing import Optional, List
import os
import uuid
from datetime import datetime
import logging
import json

class Settings(BaseSettings):
    database_url: str = os.getenv("DATABASE_URL", "postgresql+asyncpg://user:password@localhost/dbname")

    class Config:
        env_file = ".env"

settings = Settings()

# Database models
Base = declarative_base()

class ChatSessionModel(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String, unique=True, nullable=False)
    user_id = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class ChatMessageModel(Base):
    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String, nullable=False)
    role = Column(String, nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    sources = Column(Text, nullable=True)  # JSON string of sources

class DatabaseService:
    def __init__(self):
        # Set up async database engine for Neon Postgres
        self.engine = create_async_engine(settings.database_url)
        self.async_session = sessionmaker(
            self.engine, class_=AsyncSession, expire_on_commit=False
        )

    async def get_session(self, session_id: str):
        """Get a chat session from the database"""
        async with self.async_session() as session:
            try:
                result = await session.execute(
                    select(ChatSessionModel).where(ChatSessionModel.session_id == session_id)
                )
                session_obj = result.scalars().first()
                return session_obj
            except Exception as e:
                logging.error(f"Error getting session from database: {e}")
                return None

    async def save_session(self, session_id: str, user_id: Optional[str] = None):
        """Save or update a chat session in the database"""
        async with self.async_session() as session:
            try:
                # Check if session exists
                result = await session.execute(
                    select(ChatSessionModel).where(ChatSessionModel.session_id == session_id)
                )
                existing_session = result.scalars().first()

                if existing_session:
                    # Update existing session
                    existing_session.updated_at = datetime.utcnow()
                    if user_id:
                        existing_session.user_id = user_id
                else:
                    # Create new session
                    new_session = ChatSessionModel(
                        session_id=session_id,
                        user_id=user_id
                    )
                    session.add(new_session)

                await session.commit()
                return True
            except Exception as e:
                logging.error(f"Error saving session to database: {e}")
                await session.rollback()
                return False

    async def save_message(self, session_id: str, role: str, content: str, sources: Optional[List[dict]] = None):
        """Save a chat message to the database"""
        async with self.async_session() as session:
            try:
                message = ChatMessageModel(
                    session_id=session_id,
                    role=role,
                    content=content,
                    sources=json.dumps(sources) if sources else None
                )
                session.add(message)
                await session.commit()
                return True
            except Exception as e:
                logging.error(f"Error saving message to database: {e}")
                await session.rollback()
                return False

    async def get_user_interactions(self, user_id: str):
        """Get user interaction history"""
        async with self.async_session() as session:
            try:
                result = await session.execute(
                    select(ChatMessageModel)
                    .where(ChatMessageModel.session_id.in_(
                        select(ChatSessionModel.session_id).where(ChatSessionModel.user_id == user_id)
                    ))
                    .order_by(ChatMessageModel.timestamp.desc())
                    .limit(50)  # Limit to last 50 interactions
                )
                messages = result.scalars().all()
                return messages
            except Exception as e:
                logging.error(f"Error getting user interactions from database: {e}")
                return []