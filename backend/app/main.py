from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.chat import router as chat_router
from app.config import settings

app = FastAPI(
    title="Physical AI Textbook RAG API",
    version="1.0.0",
    description="RAG-powered chatbot for Physical AI and Humanoid Robotics textbook"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat_router, prefix="/api", tags=["Chat"])

@app.on_event("startup")
async def startup_event():
    pass

@app.get("/")
def root():
    return {
        "message": "Physical AI Textbook RAG API",
        "status": "running",
        "version": "1.0.0"
    }

@app.get("/health")
def health():
    return {"status": "healthy"}
