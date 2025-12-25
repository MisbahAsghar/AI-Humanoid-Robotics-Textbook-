from typing import List, Dict, Optional
import re
from app.rag.retriever import DocumentRetriever

class RAGAgent:
    def __init__(self):
        self.retriever = DocumentRetriever()
        self.greetings = {"hi", "hello", "hey", "greetings", "good morning", "good afternoon", "good evening"}
        self.thanks = {"thank you", "thanks", "thank", "appreciate"}
        self.closings = {"ok", "okay", "bye", "goodbye", "see you"}

    def query(self, question: str, selected_text: Optional[str] = None, top_k: int = 5) -> Dict:
        q_lower = question.lower().strip()

        if q_lower in self.greetings:
            return {
                "answer": "Hello! I'm here to help you explore the Physical AI and Humanoid Robotics textbook. What would you like to learn about?",
                "sources": []
            }

        if any(word in q_lower for word in self.thanks):
            return {
                "answer": "Happy to help! Feel free to ask anything else about the textbook.",
                "sources": []
            }

        if q_lower in self.closings:
            return {
                "answer": "Take care! I'll be here when you need me.",
                "sources": []
            }

        if selected_text:
            context_docs = [{"text": selected_text, "source": "Selected Text", "score": 1.0}]
        else:
            context_docs = self.retriever.retrieve(question, top_k=top_k)

        if not context_docs:
            return {
                "answer": "I couldn't find relevant information in the textbook for your question. Could you rephrase or ask about a specific chapter?",
                "sources": []
            }

        answer = self._generate_answer(question, context_docs)

        return {
            "answer": answer,
            "sources": [
                {
                    "text": doc["text"][:150] + "...",
                    "source": doc["source"],
                    "score": doc["score"]
                }
                for doc in context_docs
            ]
        }

    def _generate_answer(self, question: str, context_docs: List[Dict]) -> str:
        context_text = self._clean_text(context_docs[0]["text"])

        if len(context_text) < 50:
            return "I found a relevant section, but it's brief. Check the full chapter for complete details."

        sentences = context_text.split('. ')
        answer_text = '. '.join(sentences[:3]) + '.'

        if len(answer_text) > 400:
            answer_text = answer_text[:400] + '.'

        return answer_text

    def _clean_text(self, text: str) -> str:
        text = re.sub(r'^---.*?---\n', '', text, flags=re.DOTALL)
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)
        text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
        text = re.sub(r'\*([^*]+)\*', r'\1', text)
        text = re.sub(r'`([^`]+)`', r'\1', text)
        text = re.sub(r'^\s*[-*]\s+', '', text, flags=re.MULTILINE)
        text = re.sub(r'^\s*\d+\.\s+', '', text, flags=re.MULTILINE)
        text = re.sub(r'\n{3,}', '\n\n', text)
        text = text.strip()
        return text

    def _format_context(self, docs: List[Dict]) -> str:
        context_parts = []
        for i, doc in enumerate(docs, 1):
            context_parts.append(f"[Source {i}: {doc['source']}]\n{doc['text']}\n")
        return "\n".join(context_parts)
