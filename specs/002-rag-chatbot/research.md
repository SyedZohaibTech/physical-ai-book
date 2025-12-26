# Research: Interactive RAG Chatbot for Physical AI Textbook

## Phase 0: Research and Unknown Resolution

### Backend Setup Research

**Decision**: Use Python virtual environment with specific dependencies
**Rationale**: Python 3.11 with FastAPI provides a robust backend framework, and the specified dependencies are necessary for the RAG implementation
**Alternatives considered**: 
- Using different Python versions (decided on 3.11 for stability and compatibility)
- Different web frameworks (FastAPI chosen for async support and OpenAPI docs)

### Vector Store Integration Research

**Decision**: Implement Qdrant Cloud free tier with specific collection parameters
**Rationale**: Qdrant is well-suited for similarity search in RAG systems, and the free tier provides sufficient capacity for initial development
**Alternatives considered**:
- Other vector databases like Pinecone, Weaviate, or ChromaDB
- Self-hosted vs cloud-hosted solutions (chose cloud for simplicity)

### Database Setup Research

**Decision**: Use Neon Serverless Postgres with SQLAlchemy ORM
**Rationale**: Neon provides serverless Postgres which scales automatically and integrates well with Python applications
**Alternatives considered**:
- SQLite for simplicity (not suitable for concurrent access)
- Other cloud providers' serverless offerings
- Different ORMs like Peewee or raw SQL

### Chat Endpoint Implementation Research

**Decision**: Implement RAG pipeline with specific steps: embedding → search → prompt building → LLM call
**Rationale**: This is the standard RAG pattern that ensures responses are grounded in the textbook content
**Alternatives considered**:
- Different RAG patterns like Self-RAG or CRAG (standard RAG sufficient for requirements)

### Embedding Script Research

**Decision**: Use 500-word chunks with 50-word overlap for document splitting
**Rationale**: This chunk size balances context retention with search efficiency
**Alternatives considered**:
- Different chunk sizes (too small loses context, too large reduces precision)
- Different overlap amounts (50-word overlap balances continuity and efficiency)

### Frontend Component Research

**Decision**: React component with floating button UI pattern
**Rationale**: React is the standard for component-based UI, and the floating button pattern is common for chat interfaces
**Alternatives considered**:
- Different frameworks like Vue or vanilla JavaScript
- Different UI patterns for chat access

### Text Selection Feature Research

**Decision**: Use browser text selection APIs to detect and handle selected text
**Rationale**: Native browser APIs provide reliable text selection detection
**Alternatives considered**:
- Custom selection mechanisms
- Different interaction patterns for selected text

### Performance Optimization Research

**Decision**: Implement caching and async processing to meet 5-second response time requirement
**Rationale**: Caching frequently accessed vectors and async processing will help meet performance goals
**Alternatives considered**:
- Different optimization strategies
- Pre-computing responses (not feasible for open-ended questions)