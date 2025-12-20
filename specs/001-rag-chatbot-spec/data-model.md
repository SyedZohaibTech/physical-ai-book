# Data Model: RAG Chatbot

This document outlines the data models for the entities involved in the RAG chatbot feature, as extracted from the feature specification.

## Conversation

Represents a single exchange between a user and the chatbot, stored in Neon Postgres.

| Field       | Type        | Description                                  |
|-------------|-------------|----------------------------------------------|
| `id`        | `SERIAL`    | Unique identifier for the chat message.      |
| `user_id`   | `VARCHAR`   | Identifier for the user or session.          |
| `timestamp` | `TIMESTAMPTZ`| The time the conversation took place.        |
| `question`  | `TEXT`      | The user's original question.                |
| `answer`    | `TEXT`      | The chatbot's generated answer.              |
| `sources`   | `JSONB`     | JSON array of source documents used for the answer. |

## Text Chunk

Represents a segment of the book's content, stored in Qdrant as a vector with a corresponding payload.

| Field             | Type      | Description                                     |
|-------------------|-----------|-------------------------------------------------|
| `chunk_id`        | `UUID`    | The unique identifier for the vector in Qdrant. |
| `file_path`       | `VARCHAR` | The path to the source markdown file.           |
| `chapter`         | `VARCHAR` | The chapter or module of the content.           |
| `content_preview` | `TEXT`    | A short preview of the chunk's content.         |
