# API Contract: Embed Content Endpoint

## POST /api/embed-content

### Description
Accept content and generate embeddings to store in the vector database.

### Request
```json
{
  "file_path": "string, required - Path to the source file",
  "content": "string, required - The content to be embedded"
}
```

### Response (Success 200)
```json
{
  "success": "boolean - Whether the operation was successful",
  "chunks": "number - Number of content chunks processed and stored"
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