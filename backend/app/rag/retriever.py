from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from app.rag.embedder import Embedder
from app.config import settings
import uuid

class DocumentRetriever:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.embedder = Embedder()
        self.collection_name = "textbook_docs"
        self._ensure_collection()

    def _ensure_collection(self):
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)

        if not exists:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedder.get_dimension(),
                    distance=Distance.COSINE
                )
            )

    def store_documents(self, documents: List[Dict[str, str]]):
        texts = [doc["text"] for doc in documents]
        embeddings = self.embedder.embed_batch(texts)

        points = []
        for doc, embedding in zip(documents, embeddings):
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": doc["text"],
                    "source": doc["source"],
                    "chunk_id": doc["chunk_id"]
                }
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def retrieve(self, query: str, top_k: int = 5) -> List[Dict]:
        query_embedding = self.embedder.embed_text(query)

        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k
        )

        return [
            {
                "text": hit.payload["text"],
                "source": hit.payload["source"],
                "score": hit.score
            }
            for hit in results.points
        ]
