from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routes import chat, health

app = FastAPI(
    title="Physical AI & Humanoid Robotics Book API",
    description="API for RAG chatbot and book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
app.include_router(health.router, prefix="/api/v1", tags=["health"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)