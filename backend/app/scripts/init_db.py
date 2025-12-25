#!/usr/bin/env python3
"""Initialize Qdrant vector database with textbook content"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent))

from app.rag.loader import DocumentLoader
from app.rag.retriever import DocumentRetriever

def main():
    print("Loading documents from frontend/docs/textbook...")
    loader = DocumentLoader()
    documents = loader.load_documents()
    print(f"Loaded {len(documents)} chunks")

    print("\nInitializing Qdrant and storing embeddings...")
    retriever = DocumentRetriever()
    retriever.store_documents(documents)
    print(f"Successfully stored {len(documents)} chunks in Qdrant")

    print("\nTesting retrieval...")
    test_query = "What is Physical AI?"
    results = retriever.retrieve(test_query, top_k=3)
    print(f"\nTest query: '{test_query}'")
    print(f"Found {len(results)} relevant chunks:")
    for i, result in enumerate(results, 1):
        print(f"\n{i}. Source: {result['source']}")
        print(f"   Score: {result['score']:.4f}")
        print(f"   Text: {result['text'][:150]}...")

    print("\n✓ Database initialized successfully!")

if __name__ == "__main__":
    main()
