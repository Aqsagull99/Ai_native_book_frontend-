/**
 * RAG API Service
 *
 * Service for communicating with the RAG agent backend API.
 * Provides methods for querying the RAG agent and handling responses.
 */

// Base API URL
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000'; // Use env var or fallback to localhost

/**
 * Query the RAG agent with a user question.
 *
 * @param {string} query - The user's query/question
 * @param {Object} options - Additional options for the query
 * @param {string} options.selectedText - Optional selected text to provide context
 * @param {number} options.topK - Number of results to retrieve (default: 5)
 * @param {boolean} options.includeCitations - Whether to include citations (default: true)
 * @returns {Promise<Object>} The response from the RAG agent
 * @throws {Error} If the query fails
 */
export const queryRAGAgent = async (query, options = {}) => {
  // Validate query length (500-1000 character limit)
  if (query.length > 1000) {
    throw new Error('Query exceeds 1000 character limit. Please shorten your question.');
  }

  const {
    selectedText = null,
    topK = 5,
    includeCitations = true
  } = options;

  try {
    const response = await fetch(`${API_BASE_URL}/api/rag/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query_text: query,
        selected_text: selectedText,
        top_k: topK,
        include_citations: includeCitations,
      }),
    });

    if (!response.ok) {
      // Handle error with user-friendly message and retry option
      const errorData = await response.json();
      throw new Error(errorData.message || 'Failed to process query. Please try again.');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    // Re-throw with additional context if needed
    if (error.name === 'TypeError' && error.message.includes('fetch')) {
      throw new Error('Network error: Unable to connect to the RAG service. Please check your connection.');
    }
    throw error;
  }
};

/**
 * Check the health status of the RAG agent service.
 *
 * @returns {Promise<Object>} Health status information
 */
export const checkRAGHealth = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/api/rag/health`);

    if (!response.ok) {
      throw new Error('Health check failed');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Health check failed:', error);
    return {
      status: 'unavailable',
      timestamp: new Date().toISOString(),
      details: { error: error.message }
    };
  }
};

/**
 * Validate query parameters before sending to the backend.
 *
 * @param {string} query - The query to validate
 * @param {Object} options - Query options
 * @returns {Array} Array of validation errors, empty if valid
 */
export const validateQuery = (query, options = {}) => {
  const errors = [];

  // Check query length
  if (!query || query.trim().length === 0) {
    errors.push('Query cannot be empty');
  }

  if (query.length > 1000) {
    errors.push('Query exceeds 1000 character limit');
  }

  // Check selected text length if provided
  if (options.selectedText && options.selectedText.length > 5000) {
    errors.push('Selected text exceeds 5000 character limit');
  }

  return errors;
};

/**
 * Process a query with validation and error handling.
 *
 * @param {string} query - The user's query
 * @param {Object} options - Query options
 * @returns {Promise<Object>} The processed response or error information
 */
export const processQueryWithValidation = async (query, options = {}) => {
  // Validate the query first
  const validationErrors = validateQuery(query, options);
  if (validationErrors.length > 0) {
    throw new Error('Validation failed: ' + validationErrors.join(', '));
  }

  // Process the query
  return await queryRAGAgent(query, options);
};

// Export default API object
const RAGAPI = {
  query: queryRAGAgent,
  health: checkRAGHealth,
  validate: validateQuery,
  processWithValidation: processQueryWithValidation
};

export default RAGAPI;