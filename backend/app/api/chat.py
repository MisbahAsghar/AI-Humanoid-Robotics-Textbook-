from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict
from app.rag.agent import RAGAgent

router = APIRouter()
rag_agent = None

def get_rag_agent():
    global rag_agent
    if rag_agent is None:
        rag_agent = RAGAgent()
    return rag_agent

class ChatRequest(BaseModel):
    user_question: str
    selected_text: Optional[str] = None

class Source(BaseModel):
    text: str
    source: str
    score: float

class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        agent = get_rag_agent()
        response = agent.query(
            question=request.user_question,
            selected_text=request.selected_text
        )

        return ChatResponse(
            answer=response["answer"],
            sources=[Source(**src) for src in response["sources"]]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
