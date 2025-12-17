# Quickstart: Frontend–Backend Integration for RAG Chatbot

## Prerequisites

- Python 3.12+
- Node.js 18+ (for frontend development)
- Docusaurus project set up
- Backend RAG agent service running
- Qdrant vector database with book embeddings

## Setup

### 1. Backend Setup

1. Add CORS middleware to your FastAPI application:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],  # Adjust for your frontend URLs
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

2. Install required dependencies:

```bash
pip install python-multipart
```

### 2. Frontend Setup

1. Create the RAG API service:

```javascript
// src/services/rag-api.js
export const queryRAGAgent = async (query, options = {}) => {
  // Validate query length (500-1000 character limit)
  if (query.length > 1000) {
    throw new Error('Query exceeds 1000 character limit. Please shorten your question.');
  }

  const response = await fetch('http://localhost:8000/rag-agent/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      query_text: query,
      selected_text: options.selectedText || null, // Include optional selected text as context
      top_k: options.topK || 5,
      include_citations: options.includeCitations || true,
    }),
  });

  if (!response.ok) {
    // Handle error with user-friendly message and retry option
    const errorData = await response.json();
    throw new Error(errorData.message || 'Failed to process query. Please try again.');
  }

  return response.json();
};
```

2. Create the dedicated chatbot component (always visible on pages):

```javascript
// src/components/RagChatbot.js
import React, { useState } from 'react';
import { queryRAGAgent } from '../services/rag-api';

const RagChatbot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState(''); // Track selected text

  // Function to capture selected text
  const handleTextSelection = () => {
    const selected = window.getSelection().toString().trim();
    if (selected) {
      setSelectedText(selected);
      // Optionally pre-fill the query field with selected text
      if (!query) {
        setQuery(selected);
      }
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!query.trim()) return;

    setLoading(true);
    setError(null);

    try {
      const result = await queryRAGAgent(query, {
        selectedText: selectedText,
        includeCitations: true
      });
      setResponse(result);
      // Clear selected text after successful query
      setSelectedText('');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  // Add event listener for text selection
  React.useEffect(() => {
    const handleSelection = () => {
      setTimeout(handleTextSelection, 0); // Delay to allow selection to complete
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleRetry = () => {
    if (response && response.query_text) {
      setQuery(response.query_text);
      setResponse(null);
      setError(null);
      handleSubmit({ preventDefault: () => {} });
    }
  };

  return (
    <div className="rag-chatbot">
      <h3>Ask about this book</h3>
      <form onSubmit={handleSubmit}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about the book content (max 1000 characters)..."
          disabled={loading}
          maxLength={1000} // Enforce character limit
        />
        <small>{query.length}/1000 characters</small>
        <button type="submit" disabled={loading}>
          {loading ? 'Thinking...' : 'Ask'}
        </button>
      </form>

      {loading && <div className="loading">Loading response...</div>}

      {error && (
        <div className="error">
          <p>{error}</p>
          <button onClick={() => setError(null)}>Dismiss</button>
        </div>
      )}

      {response && (
        <div className="response">
          <h4>Answer:</h4>
          <p>{response.answer}</p>

          {response.retrieved_chunks && response.retrieved_chunks.length > 0 && (
            <div className="sources">
              <h5>Sources:</h5>
              <ul>
                {response.retrieved_chunks.map((chunk, index) => (
                  <li key={index}>
                    <a href={chunk.metadata.url} target="_blank" rel="noopener noreferrer">
                      {chunk.metadata.title}
                    </a>
                    <p>{chunk.content.substring(0, 100)}...</p>
                  </li>
                ))}
              </ul>
            </div>
          )}

          <button onClick={handleRetry}>Ask again</button>
        </div>
      )}
    </div>
  );
};

export default RagChatbot;
```

### 3. Integration with Docusaurus

1. Add the dedicated chatbot component to your Docusaurus pages (always visible):

```javascript
// In your Docusaurus layout or page
import RagChatbot from '@site/src/components/RagChatbot';

function MyLayout({ children }) {
  return (
    <div>
      <main>{children}</main>
      <aside className="chatbot-sidebar">
        {/* Always visible chatbot component */}
        <RagChatbot />
      </aside>
    </div>
  );
}
```

### 4. Running the Services

1. Start the backend RAG service:

```bash
cd backend
uvicorn main:app --reload --port 8000
```

2. Start the Docusaurus frontend:

```bash
cd frontend  # or wherever your Docusaurus project is
npm start
```

## API Endpoints

- `POST /rag-agent/query` - Submit a query to the RAG agent with source citations
- `GET /rag-agent/health` - Check service health

## Configuration

### Environment Variables

Backend (.env):
```
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
GEMINI_API_KEY=your_gemini_api_key
```

Frontend (.env):
```
REACT_APP_RAG_API_URL=http://localhost:8000
```

## Testing

Run a simple test to verify the integration:

```javascript
// Test the RAG agent integration
import { queryRAGAgent } from './services/rag-api';

async function testRAG() {
  try {
    const response = await queryRAGAgent("What is artificial intelligence?", {
      selectedText: "Artificial intelligence is a branch of computer science",
      includeCitations: true
    });
    console.log("RAG Response:", response);
    console.log("Answer:", response.answer);
    console.log("Sources with citations:", response.retrieved_chunks.length);
  } catch (error) {
    console.error("RAG query failed:", error);
  }
}

testRAG();
```