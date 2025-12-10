from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Any

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, str]

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify API availability
    """
    import datetime

    # In a real implementation, you would check actual service connections
    services_status = {
        "vector_db": "connected",  # Would check Qdrant connection
        "database": "connected",   # Would check Neon Postgres connection
        "openai": "connected"      # Would check OpenAI connection
    }

    return HealthResponse(
        status="healthy",
        timestamp=datetime.datetime.now().isoformat(),
        services=services_status
    )