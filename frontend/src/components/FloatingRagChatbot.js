/**
 * Professional Floating RAG Chatbot Component
 *
 * A floating chatbot interface with chat bubbles, typing animation,
 * source citations, and professional design for documentation-based learning.
 */

import React, { useState, useEffect, useRef } from 'react';
import { FiMessageCircle, FiX, FiSend, FiBook, FiChevronDown, FiChevronUp, FiLoader } from 'react-icons/fi';
import RAGAPI from '../services/rag-api';

const FloatingRagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, type: 'bot', content: 'Ask me anything about this book! I can help you understand the concepts and find relevant information.' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [sourcesExpanded, setSourcesExpanded] = useState({});
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Function to capture selected text
  const handleTextSelection = () => {
    const selected = window.getSelection().toString().trim();
    if (selected) {
      setSelectedText(selected);
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
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), type: 'user', content: inputValue.trim() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const result = await RAGAPI.processWithValidation(userMessage.content, {
        selectedText: selectedText,
        includeCitations: true
      });

      const botMessage = {
        id: Date.now() + 1,
        type: 'bot',
        content: result.answer,
        sources: result.retrieved_chunks || []
      };

      setMessages(prev => [...prev, botMessage]);
      setSelectedText(''); // Clear selected text after successful query
    } catch (err) {
      const errorMessage = {
        id: Date.now() + 1,
        type: 'bot',
        content: `I encountered an error: ${err.message}. Please try again.`
      };
      setMessages(prev => [...prev, errorMessage]);
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleSourceExpansion = (messageId) => {
    setSourcesExpanded(prev => ({
      ...prev,
      [messageId]: !prev[messageId]
    }));
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  };

  // Typing indicator component
  const TypingIndicator = () => (
    <div className="typing-indicator">
      <div className="typing-dot"></div>
      <div className="typing-dot"></div>
      <div className="typing-dot"></div>
    </div>
  );

  return (
    <div className="floating-rag-chatbot">
      {/* Floating Button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? <FiX size={20} /> : <FiMessageCircle size={20} />}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Documentation Assistant</h3>
            <button
              className="close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              <FiX size={18} />
            </button>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.type} ${message.type === 'bot' && message.sources?.length > 0 ? 'with-sources' : ''}`}
              >
                <div className="message-content">
                  {message.content}
                </div>

                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <button
                      className="sources-toggle"
                      onClick={() => toggleSourceExpansion(message.id)}
                    >
                      <FiBook size={14} />
                      {sourcesExpanded[message.id] ? 'Hide sources' : 'Show sources'}
                      {sourcesExpanded[message.id] ? <FiChevronUp size={14} /> : <FiChevronDown size={14} />}
                    </button>

                    {sourcesExpanded[message.id] && (
                      <div className="sources-list">
                        {message.sources.map((source, index) => (
                          <div key={index} className="source-item">
                            <a
                              href={source.metadata?.url}
                              target="_blank"
                              rel="noopener noreferrer"
                              className="source-link"
                            >
                              {source.metadata?.title || 'Source'}
                            </a>
                            <p className="source-excerpt">
                              {source.content.substring(0, 100)}...
                            </p>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className="message bot">
                <div className="message-content">
                  <TypingIndicator />
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form className="chat-input-form" onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask anything about this chapter..."
              disabled={isLoading}
              maxLength={1000}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              aria-label="Send message"
            >
              {isLoading ? <FiLoader className="loading-spinner" size={18} /> : <FiSend size={18} />}
            </button>
          </form>

          {error && (
            <div className="chat-error">
              <p>{error}</p>
              <button onClick={() => setError(null)}>Dismiss</button>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default FloatingRagChatbot;