# Quickstart: RAG Agent with OpenAI Agents SDK

## Prerequisites

1. **Environment Setup**
   - Python 3.12+
   - Access to Qdrant Cloud instance with `ai_native_book` collection
   - Google AI API key with Gemini 1.5 Flash access
   - Existing embedded content in the Qdrant collection

2. **Environment Variables**
   ```bash
   export GEMINI_API_KEY="your-google-ai-api-key"
   export GEMINI_MODEL_NAME="gemini-1.5-flash"
   export QDRANT_URL="your-qdrant-cloud-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   ```

## Installation

1. **Install Dependencies**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Verify Setup**
   ```bash
   python -c "import openai_agents_sdk, google.generativeai; print('Dependencies available')"
   ```

## Running the Agent

1. **Start the Agent Service**
   ```bash
   cd backend
   uvicorn main:app --reload --port 8000
   ```

2. **Test the Agent via API**
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{
       "query_text": "What are the key concepts in AI and machine learning?",
       "top_k": 5,
       "include_citations": true
     }'
   ```

3. **Run Agent Validation**
   ```bash
   cd backend
   python test_agent.py
   ```

4. **Direct Agent Testing**
   ```bash
   cd backend
   python -c "
   from src.rag_agent.agent import create_rag_agent
   agent = create_rag_agent()
   response = agent.process_query('your query here', top_k=5)
   print(response)
   "
   ```

## Expected Output

The agent will return responses in the following format:
```json
{
  "query_text": "What are the key concepts in AI and machine learning?",
  "answer": "The key concepts in AI and machine learning include...",
  "retrieved_chunks": [
    {
      "content": "Artificial intelligence encompasses various subfields...",
      "similarity_score": 0.89,
      "metadata": {
        "url": "https://example.com/page1",
        "title": "AI Fundamentals",
        "chunk_index": 1,
        "source_metadata": {},
        "created_at": "2025-12-16T05:00:00Z"
      }
    }
  ],
  "confidence_score": 0.92,
  "execution_time": 1.234,
  "timestamp": "2025-12-16T05:00:00Z"
}
```

## Troubleshooting

- **Connection Issues**: Verify GEMINI_API_KEY and QDRANT_URL are correctly set
- **Authentication Issues**: Ensure your Google AI API key has proper permissions for Gemini 1.5 Flash
- **No Results**: Confirm the `ai_native_book` collection has embedded content
- **Performance Issues**: If queries are slow, check your network connection to Qdrant Cloud
- **MCP Configuration**: Verify context 7 MCP servers are properly configured for agent runtime