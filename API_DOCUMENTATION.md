# RAG Agent API Documentation

## Overview

The RAG (Retrieval-Augmented Generation) Agent API provides endpoints for querying book content and receiving grounded responses with source citations. The API integrates with vector databases to retrieve relevant content and uses language models to generate contextually relevant answers.

## Base URL

The API is served at `/api/rag` under your backend server. For local development, this is typically:
`http://localhost:8000/api/rag`

## Endpoints

### POST /query

Process a user query through the RAG agent and return a grounded response based on book content with source citations and links.

#### Request

**Content-Type:** `application/json`

**Body Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `query_text` | string | Yes | The user's query text (1-1000 characters) |
| `selected_text` | string | No | Optional selected text from the book content to provide context in the query (max 5000 characters) |
| `top_k` | integer | No | Number of results to retrieve (default: 5, range: 1-20) |
| `include_citations` | boolean | No | Whether to include source citations and links in the response (default: true) |

**Example Request:**
```json
{
  "query_text": "What are the main principles of AI agents?",
  "selected_text": "The chapter discusses various approaches to implementing AI agents.",
  "top_k": 5,
  "include_citations": true
}
```

#### Response

**Success Response (200 OK):**

| Field | Type | Description |
|-------|------|-------------|
| `query_text` | string | The original query text submitted by the user |
| `answer` | string | The generated answer based on retrieved content |
| `retrieved_chunks` | array | List of content chunks used to generate the answer |
| `confidence_score` | number | Overall confidence in the answer (0-1) |
| `execution_time` | number | Query execution time in seconds |
| `timestamp` | string | ISO 8601 formatted timestamp of when the query was processed |

**Content Chunk Structure:**

| Field | Type | Description |
|-------|------|-------------|
| `content` | string | The actual content text retrieved |
| `similarity_score` | number | Cosine similarity score (0-1) indicating relevance |
| `metadata` | object | Metadata about the source including title, URL, etc. |
| `rank` | integer | Position in the results (1-indexed) |

**Example Response:**
```json
{
  "query_text": "What are the main principles of AI agents?",
  "answer": "The main principles of AI agents include autonomous operation, goal-oriented behavior, and the ability to perceive and act upon their environment...",
  "retrieved_chunks": [
    {
      "content": "AI agents operate autonomously to achieve specific goals...",
      "similarity_score": 0.95,
      "metadata": {
        "title": "Introduction to AI Agents",
        "url": "https://example.com/book/chapter1",
        "section": "Chapter 1"
      },
      "rank": 1
    }
  ],
  "confidence_score": 0.85,
  "execution_time": 1.23,
  "timestamp": "2025-12-16T10:30:00.123456"
}
```

**Error Responses:**

- **400 Bad Request**: Validation error (e.g., query exceeds character limit)
  ```json
  {
    "detail": {
      "error": "VALIDATION_ERROR",
      "message": "Query exceeds 1000 character limit. Please shorten your question."
    }
  }
  ```

- **500 Internal Server Error**: Internal processing error
  ```json
  {
    "detail": {
      "error": "QUERY_PROCESSING_ERROR",
      "message": "Failed to process query due to internal error. Please try again."
    }
  }
  ```

### GET /health

Check the health status of the RAG agent service.

#### Response

**Success Response (200 OK):**

| Field | Type | Description |
|-------|------|-------------|
| `status` | string | Health status ("healthy" or "unavailable") |
| `timestamp` | string | ISO 8601 formatted timestamp |
| `details` | object | Additional health details |

**Example Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T10:30:00.123456",
  "details": {
    "rag_agent_initialized": true
  }
}
```

## Error Handling

The API uses standard HTTP status codes and returns detailed error messages in the response body:

- **400 Bad Request**: Request validation failed
- **500 Internal Server Error**: Server-side processing error

## Rate Limiting

The API does not currently implement rate limiting. In production environments, consider implementing rate limiting to prevent abuse.

## Authentication

This API does not require authentication. In production environments, consider implementing authentication if needed.

## CORS

The API is configured to allow cross-origin requests from all origins during development. In production, configure specific allowed origins.