# Data Model: Interactive RAG Chatbot for Physical AI Textbook

## Entities

### Conversation
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to user, optional for anonymous users)
- **created_at**: DateTime (Timestamp of conversation creation)
- **updated_at**: DateTime (Timestamp of last activity)

### Message
- **id**: UUID (Primary Key)
- **conversation_id**: UUID (Foreign Key to Conversation)
- **role**: String (Either "user" or "assistant")
- **content**: Text (The message content)
- **created_at**: DateTime (Timestamp of message creation)
- **sources**: JSON (Optional array of source references from textbook)

### Textbook Content (Vector Store)
- **id**: UUID (Primary Key)
- **file_path**: String (Path to the source document)
- **module**: String (Module or section identifier)
- **title**: String (Title of the content chunk)
- **chunk_index**: Integer (Sequential index within the document)
- **content**: Text (The actual content chunk)
- **vector**: Array (Embedding vector representation)
- **metadata**: JSON (Additional metadata like page numbers, section headings)

## Relationships
- Conversation (1) ←→ (Many) Message (One-to-many relationship)
- Textbook Content exists independently in vector store

## Validation Rules
- Conversation must have at least one message
- Message must have a valid role (user/assistant)
- Message must belong to an existing conversation
- Textbook content chunks must not exceed 500 words
- Textbook content vectors must be 1536 dimensions (OpenAI embedding size)

## State Transitions
- Conversation starts when first message is created
- Conversation updates when new messages are added
- Conversation may be marked as inactive after period of inactivity