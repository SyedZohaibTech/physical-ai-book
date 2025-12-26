# API Contract: Chat Endpoint

## POST /api/chat

### Description
Process user message and return AI-generated response based on textbook content using RAG (Retrieval-Augmented Generation).

### Request
```json
{
  "message": "string, required - The user's message or question",
  "context": "string, optional - Additional context (e.g., selected text)",
  "conversation_id": "string, optional - Existing conversation ID to continue conversation"
}
```

### Response (Success 200)
```json
{
  "response": "string - The AI-generated response",
  "sources": [
    {
      "file_path": "string - Path to source document",
      "module": "string - Module or section identifier",
      "title": "string - Title of the content chunk",
      "chunk_index": "integer - Sequential index within the document",
      "content": "string - Excerpt from the source content"
    }
  ],
  "conversation_id": "string - New or existing conversation ID"
}
```

### Response (Error 400)
```json
{
  "error": "string - Error message explaining the issue"
}
```

### Response (Error 500)
```json
{
  "error": "string - Error message indicating server-side issue"
}
```

### Performance Requirements
- Response time: < 5 seconds
- Should include loading indicators for client during processing