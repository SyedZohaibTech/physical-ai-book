from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import logging
import os
from dotenv import load_dotenv
from typing import Optional, List, Dict, Any
import asyncio

# Load environment variables
load_dotenv()

# Import our modules
from vector_store import VectorStore
from embeddings import generate_embedding
from database import get_session, Conversation, Message
from sqlalchemy.orm import Session

# Initialize FastAPI app
app = FastAPI(title="RAG Chatbot API", description="API for the Interactive RAG Chatbot for Physical AI Textbook")

# Add CORS middleware to allow requests from localhost:3000
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],  # Allow requests from React dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize vector store
vector_store = VectorStore()

# Pydantic models for request/response
class ChatRequest(BaseModel):
    message: str
    context: Optional[str] = None
    conversation_id: Optional[str] = None

class Source(BaseModel):
    file_path: str
    module: str
    title: str
    chunk_index: int
    content: str

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    conversation_id: str

class HealthResponse(BaseModel):
    status: str
    services: Dict[str, bool]

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Process user message and return AI-generated response based on textbook content using RAG.
    """
    try:
        logger.info(f"Received chat request: {request.message[:50]}...")

        # Generate embedding for the user's message
        query_embedding = await generate_embedding(request.message)

        # Search for similar content in the vector store
        search_results = vector_store.search_similar(query_embedding)

        # Build context from search results
        context_parts = []
        sources = []

        for result in search_results:
            context_parts.append(result['content'])
            sources.append(Source(
                file_path=result['file_path'],
                module=result['module'],
                title=result['title'],
                chunk_index=result['chunk_index'],
                content=result['content']
            ))

        # Combine retrieved context with the original message
        full_context = "\n".join(context_parts)
        if request.context:
            full_context = f"Context from selection: {request.context}\n\nRetrieved content: {full_context}"

        # Generate a simple response based on the retrieved content
        if context_parts:
            response_text = f"Based on the context: {full_context[:500]}..., here's the answer to your question: {request.message}"
        else:
            response_text = "I couldn't find relevant information in the textbook to answer your question."

        # Generate a conversation ID if not provided
        conversation_id = request.conversation_id
        if not conversation_id:
            import uuid
            conversation_id = str(uuid.uuid4())

        # Save the conversation to the database
        with get_session() as session:
            # Get or create conversation
            conversation = session.query(Conversation).filter(Conversation.id == conversation_id).first()
            if not conversation:
                conversation = Conversation(id=conversation_id)
                session.add(conversation)
                session.commit()

            # Save user message
            user_message = Message(
                conversation_id=conversation_id,
                role="user",
                content=request.message
            )
            session.add(user_message)

            # Save assistant response
            assistant_message = Message(
                conversation_id=conversation_id,
                role="assistant",
                content=response_text
            )
            session.add(assistant_message)

            session.commit()

        return ChatResponse(
            response=response_text,
            sources=sources,
            conversation_id=conversation_id
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.get("/api/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the application and its dependencies.
    """
    try:
        # Check Qdrant connection
        qdrant_ok = vector_store.health_check()

        # Check Postgres connection
        postgres_ok = False
        try:
            with get_session() as session:
                # Simple query to test connection
                session.execute("SELECT 1")
                postgres_ok = True
        except Exception as e:
            logger.error(f"Postgres health check failed: {str(e)}")
            postgres_ok = False

        services_status = {
            "qdrant": qdrant_ok,
            "postgres": postgres_ok
        }

        overall_status = "ok" if all(services_status.values()) else "error"

        return HealthResponse(
            status=overall_status,
            services=services_status
        )
    except Exception as e:
        logger.error(f"Health check error: {str(e)}")
        return HealthResponse(
            status="error",
            services={
                "qdrant": False,
                "postgres": False
            },
            error=str(e)
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)