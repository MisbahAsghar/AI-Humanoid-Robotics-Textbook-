from pydantic_settings import BaseSettings
from typing import List

class Settings(BaseSettings):
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
    OPENAI_API_KEY: str = ""
    HF_API_KEY: str = ""
    CORS_ORIGINS: List[str] = ["https://text-book-physical-ai-humanoid-robo.vercel.app", "http://localhost:3000"]

    class Config:
        env_file = ".env"

settings = Settings()
