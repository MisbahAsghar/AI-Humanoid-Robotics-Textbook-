# Physical AI Textbook RAG Backend

## What This Is

A RAG (Retrieval-Augmented Generation) chatbot for the Physical AI and Humanoid Robotics textbook. Answers questions using ONLY content from the book chapters.

## RAG Explanation

1. **Load**: Parse markdown chapters into text chunks
2. **Embed**: Convert chunks to vectors using sentence-transformers
3. **Store**: Save vectors in Qdrant vector database
4. **Retrieve**: Find relevant chunks for user questions
5. **Generate**: Use OpenAI GPT-3.5-turbo to answer from retrieved context

## Free Services Used

- **Qdrant Cloud** - Free tier vector database (1GB storage)
- **Neon** - Free tier PostgreSQL database
- **OpenAI API** - GPT-3.5-turbo (pay-as-you-go, low cost)
- **Sentence Transformers** - Free local embeddings (all-MiniLM-L6-v2)

## Setup

1. Create virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Configure environment:
```bash
cp .env.example .env
# Edit .env with your API keys
```

4. Load documents into Qdrant:
```bash
python -m app.scripts.init_db
```

5. Run server:
```bash
uvicorn app.main:app --reload --port 8000
```

## API Endpoints

- `POST /api/chat` - Send question, get answer with sources
  - Body: `{"user_question": "...", "selected_text": "..." (optional)}`
  - If `selected_text` provided, answers ONLY from that text
  - Otherwise uses RAG pipeline

## Deployment

### Hugging Face Spaces

Deploy as a Docker Space:

1. Create `Dockerfile` in backend/
2. Push to Hugging Face Spaces
3. Add secrets in Space settings

### Vercel (Alternative)

Not recommended for RAG backend due to cold starts and resource limits.

## Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI app
│   ├── config.py            # Settings
│   ├── rag/
│   │   ├── loader.py        # Document loading
│   │   ├── embedder.py      # Sentence transformers
│   │   ├── retriever.py     # Qdrant retrieval
│   │   └── agent.py         # OpenAI RAG agent
│   └── api/
│       └── chat.py          # Chat endpoints
├── requirements.txt
└── .env.example
```
