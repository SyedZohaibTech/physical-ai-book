# API Contract: Health Check Endpoint

## GET /api/health

### Description
Check the health status of the application and its dependencies.

### Response (Success 200)
```json
{
  "status": "string - Overall status ('ok' if healthy)",
  "services": {
    "qdrant": "boolean - Whether Qdrant vector store is accessible",
    "postgres": "boolean - Whether Postgres database is accessible"
  }
}
```

### Response (Error 500)
```json
{
  "status": "string - 'error' if unhealthy",
  "services": {
    "qdrant": "boolean - Whether Qdrant vector store is accessible",
    "postgres": "boolean - Whether Postgres database is accessible"
  },
  "error": "string - Error message indicating the issue"
}
```