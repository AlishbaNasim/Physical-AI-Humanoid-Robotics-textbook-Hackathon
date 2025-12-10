"""
Test script to verify the RAG chatbot functionality
"""
import asyncio
import os
import sys
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from services.chat_service import ChatService

async def test_chat_functionality():
    """Test the chat functionality with sample queries"""
    print("Testing RAG Chatbot functionality...")

    # Initialize the chat service
    chat_service = ChatService()

    # Test queries
    test_queries = [
        "What is ROS 2?",
        "How does ROS 2 relate to humanoid robotics?",
        "What are the key features of ROS 2?"
    ]

    print("\nRunning test queries...")
    for i, query in enumerate(test_queries, 1):
        print(f"\nTest {i}: Query: '{query}'")
        try:
            result = await chat_service.process_chat(
                message=query,
                session_id=f"test-session-{i}",
                user_id="test-user"
            )
            print(f"Response: {result['response'][:200]}...")  # Show first 200 chars
            print(f"Sources: {len(result['sources'])} found")
        except Exception as e:
            print(f"Error in test {i}: {str(e)}")

    print("\nChat functionality test completed!")

if __name__ == "__main__":
    asyncio.run(test_chat_functionality())