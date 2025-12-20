# Research: RAG Chatbot

## Technology Stack

**Decision**: The project will use the following technology stack:
- **Backend**: FastAPI (Python 3.11)
- **Vector Database**: Qdrant Cloud
- **Chat History & Metadata**: Neon Serverless Postgres
- **Frontend**: React/TypeScript within the existing Docusaurus site
- **AI Services**: OpenAI API (`text-embedding-3-small` and `gpt-4o-mini`)

**Rationale**: This stack was chosen based on the detailed requirements outlined in the project constitution.
- **FastAPI** is a high-performance Python web framework, ideal for building the API backend.
- **Qdrant** is a scalable, open-source vector database well-suited for semantic search in a RAG pipeline.
- **Neon** provides a serverless Postgres option that is easy to manage and cost-effective for storing chat history.
- **React/TypeScript** allows for seamless integration of the chatbot component into the existing Docusaurus website.
- **OpenAI** provides state-of-the-art models for embeddings and chat generation.

**Alternatives Considered**:
- **Backend Frameworks**: Flask and Django were considered but FastAPI was chosen for its modern async capabilities and performance.
- **Vector Databases**: Pinecone and Weaviate were evaluated, but Qdrant's open-source nature and flexible deployment options were preferred.
- **Frontend Frameworks**: While other frameworks could have been used, integrating a new framework would have been more complex than building a React component for the existing Docusaurus site.
