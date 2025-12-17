# Local Development Setup for RAG Chatbot Integration

## Prerequisites

- Python 3.12+
- Node.js 18+ (for frontend development)
- Docker (for vector database - optional)
- Qdrant Cloud account (for vector storage)

## Backend Setup

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up environment variables:**
   Create a `.env` file in the backend directory with the following:
   ```env
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cluster_url
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   ```

3. **Start the backend server:**
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

## Frontend Setup

1. **Install Node.js dependencies:**
   ```bash
   cd frontend
   npm install
   ```

2. **Set up environment variables:**
   Create a `.env` file in the frontend directory with the following:
   ```env
   REACT_APP_RAG_API_URL=http://localhost:8000
   ```

3. **Start the frontend development server:**
   ```bash
   npm start
   ```

## API Endpoints

The RAG chatbot integration exposes the following endpoints:

### POST /api/rag/query
Process a user query through the RAG agent and return a grounded response.

**Request Body:**
```json
{
  "query_text": "Your question here (max 1000 characters)",
  "selected_text": "Optional selected text from book content (max 5000 characters)",
  "top_k": 5,
  "include_citations": true
}
```

**Response:**
```json
{
  "query_text": "Your question here",
  "answer": "The generated answer",
  "retrieved_chunks": [
    {
      "content": "Relevant content chunk",
      "similarity_score": 0.95,
      "metadata": {
        "title": "Source Title",
        "url": "Source URL",
        "section": "Section Name"
      },
      "rank": 1
    }
  ],
  "confidence_score": 0.85,
  "execution_time": 1.23,
  "timestamp": "2025-12-16T10:30:00.123456"
}
```

### GET /api/rag/health
Check the health status of the RAG agent service.

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-16T10:30:00.123456",
  "details": {
    "rag_agent_initialized": true
  }
}
```

## Testing the Integration

1. **Start both backend and frontend servers**
2. **Navigate to the frontend application**
3. **Use the RAG chatbot component to ask questions about book content**
4. **Verify that responses include source citations and links**

## Troubleshooting

### CORS Issues
- Ensure the backend CORS configuration allows the frontend origin
- Check that `REACT_APP_RAG_API_URL` points to the correct backend URL

### Vector Database Connection
- Verify Qdrant credentials in environment variables
- Check that the vector collection "ai_native_book" exists

### API Response Issues
- Check backend logs for error messages
- Verify OpenAI and Cohere API keys are valid