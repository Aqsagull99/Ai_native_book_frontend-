/**
 * RAG Chatbot Component
 *
 * A dedicated chat interface component that's always visible on pages,
 * allowing users to ask questions about the book content and receive
 * responses with source citations.
 */

import React, { useState, useEffect } from 'react';
import RAGAPI from '../services/rag-api';

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
      const result = await RAGAPI.processWithValidation(query, {
        selectedText: selectedText,
        includeCitations: true
      });
      setResponse(result);
      // Clear the query input after successful submission
      setQuery('');
      // Clear selected text after successful query
      setSelectedText('');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleSelection = () => {
      setTimeout(handleTextSelection, 0); // Delay to allow selection to complete
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [query]);

  const handleRetry = () => {
    // For "Ask again", we just want to clear the response and allow new input
    // Don't re-use the previous query - let user type a new question
    setResponse(null);
    setError(null);
    // Optionally clear any error state but keep the input ready for new query
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