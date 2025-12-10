import asyncio
import os
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.services.chat_service import ChatService

async def ingest_book_content(content_dir: str):
    """Ingest all book content from markdown files into the vector database"""
    print(f"Starting content ingestion from: {content_dir}")

    chat_service = ChatService()

    # Walk through the content directory to find all markdown files
    content_path = Path(content_dir)

    if not content_path.exists():
        print(f"Content directory does not exist: {content_dir}")
        return

    # Find all markdown files
    md_files = list(content_path.rglob("*.md"))

    print(f"Found {len(md_files)} markdown files to process")

    for i, md_file in enumerate(md_files, 1):
        print(f"Processing ({i}/{len(md_files)}): {md_file}")

        try:
            # Read the markdown file
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Create content ID from file path
            relative_path = md_file.relative_to(content_path)
            content_id = str(relative_path).replace('/', '_').replace('\\', '_').replace('.md', '')

            # Extract title from the first line if it's a markdown header
            lines = content.split('\n')
            title = "Untitled"
            for line in lines:
                if line.startswith('# '):
                    title = line[2:].strip()
                    break

            # Determine module from path
            module_parts = str(relative_path).split(os.sep)
            module = module_parts[0] if module_parts else "unknown"

            # In a real implementation, you might want to chunk the content
            # For now, we'll ingest the entire file as one piece
            vector_id = await chat_service.embed_content(
                content_id=content_id,
                title=title,
                content=content,
                module=module
            )

            print(f"  - Embedded as: {vector_id}")

        except Exception as e:
            print(f"  - Error processing {md_file}: {str(e)}")

    print("Content ingestion completed!")

def main():
    if len(sys.argv) != 2:
        print("Usage: python -m src.cli.ingestion <content_directory>")
        sys.exit(1)

    content_dir = sys.argv[1]

    if not os.path.exists(content_dir):
        print(f"Error: Directory does not exist: {content_dir}")
        sys.exit(1)

    # Run the async function
    asyncio.run(ingest_book_content(content_dir))

if __name__ == "__main__":
    main()