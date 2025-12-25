from pathlib import Path
from typing import List, Dict
import re

class DocumentLoader:
    def __init__(self, docs_path: str = "../frontend/docs"):
        self.docs_path = Path(docs_path)
        self.chunk_size = 1000
        self.chunk_overlap = 200

    def load_documents(self) -> List[Dict[str, str]]:
        documents = []

        for md_file in self.docs_path.rglob("*.md"):
            if self._should_skip(md_file):
                continue

            content = self._read_markdown(md_file)
            chunks = self._chunk_text(content)

            for idx, chunk in enumerate(chunks):
                documents.append({
                    "text": chunk,
                    "source": str(md_file.relative_to(self.docs_path)),
                    "chunk_id": idx
                })

        return documents

    def _should_skip(self, file_path: Path) -> bool:
        skip_dirs = {"node_modules", "build", ".docusaurus"}
        return any(skip_dir in file_path.parts for skip_dir in skip_dirs)

    def _read_markdown(self, file_path: Path) -> str:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        return content.strip()

    def _chunk_text(self, text: str) -> List[str]:
        chunks = []
        lines = text.split('\n')
        current_chunk = []
        current_size = 0

        for line in lines:
            line_size = len(line)

            if current_size + line_size > self.chunk_size and current_chunk:
                chunks.append('\n'.join(current_chunk))
                overlap_lines = current_chunk[-3:] if len(current_chunk) > 3 else current_chunk
                current_chunk = overlap_lines + [line]
                current_size = sum(len(l) for l in current_chunk)
            else:
                current_chunk.append(line)
                current_size += line_size

        if current_chunk:
            chunks.append('\n'.join(current_chunk))

        return chunks
