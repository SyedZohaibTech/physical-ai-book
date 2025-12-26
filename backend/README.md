# Interactive RAG Chatbot for Physical AI Textbook - Backend

This is the backend component of the Interactive RAG Chatbot for Physical AI Textbook. It provides a FastAPI-based API that handles chat requests, manages vector storage with Qdrant, and connects to a Postgres database for conversation history.

## Setup Instructions

1. Create Qdrant Cloud account, get URL and API key
2. Create Neon Postgres database, get connection URL
3. Get OpenAI API key
4. Copy .env.example to .env, fill in keys
5. Install: pip install -r requirements.txt
6. Embed content: python embed_book.py
7. Run server: uvicorn main:app --reload --port 8000
8. Test: curl localhost:8000/api/health

## API Endpoints

- `POST /api/chat` - Process user messages and return AI-generated responses
- `GET /api/health` - Check the health status of services

## Architecture

The backend follows a RAG (Retrieval-Augmented Generation) pattern:
1. User sends a message to the chat endpoint
2. System generates an embedding for the message
3. System searches the vector store for similar content
4. System builds a prompt with the retrieved content
5. System calls OpenAI's GPT-3.5-turbo to generate a response
6. System saves the conversation to the database
7. System returns the response to the user with source citations