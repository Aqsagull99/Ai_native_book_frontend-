# Data Model: RAG Agent with OpenAI Agents SDK

## Entities

### AgentRequest
- **query_text**: string - The natural language query from the user
- **top_k**: integer - Number of results to retrieve from vector database (default: 5)
- **temperature**: float - Temperature parameter for LLM generation (default: 0.7)
- **max_tokens**: integer - Maximum tokens in response (default: 1000)
- **conversation_context**: optional object - Previous conversation turns for context
- **include_citations**: boolean - Whether to include source citations (default: true)

### AgentResponse
- **query_text**: string - The original query text
- **answer**: string - The generated answer based on retrieved content
- **retrieved_chunks**: array of ContentChunk objects - Content chunks used in answer generation
- **confidence_score**: float - Overall confidence in the answer (0-1)
- **execution_time**: float - Time taken to process the query in seconds
- **timestamp**: datetime - When the query was processed

### ContentChunk
- **content**: string - The actual content text
- **similarity_score**: float - Similarity score from vector search (0-1)
- **metadata**: object - Full metadata from the stored chunk
  - **url**: string - Source URL of the content
  - **title**: string - Title of the source document
  - **chunk_index**: integer - Index of this chunk within the source
  - **source_metadata**: object - Additional source metadata
  - **created_at**: datetime - When the chunk was created

### AgentState
- **session_id**: string - Unique identifier for the conversation session
- **conversation_history**: array of ConversationTurn objects - Complete conversation history
- **current_context**: string - Current context for the agent
- **last_accessed**: datetime - When this state was last accessed

### ConversationTurn
- **turn_id**: integer - Sequential ID for the turn (0-indexed)
- **query**: string - The user's query
- **response**: string - The agent's response
- **retrieved_sources**: array of ContentChunk objects - Sources used for this turn
- **timestamp**: datetime - When this turn occurred

### AgentMetrics
- **total_queries**: integer - Number of queries processed
- **successful_queries**: integer - Number of queries that returned results
- **avg_response_time**: float - Average query response time in seconds
- **avg_confidence_score**: float - Average confidence in responses (0-1)
- **metadata_accuracy**: float - Percentage of results with complete metadata (0-100)
- **relevance_accuracy**: float - Percentage of results that are semantically relevant (0-100)
- **consistency_rate**: float - Percentage of consistent results across repeated queries (0-100)

### AgentResult
- **agent_request**: AgentRequest - The original request that was processed
- **agent_response**: AgentResponse - The response that was generated
- **metadata_preserved**: boolean - Whether all metadata fields are present and valid
- **relevance_score**: float - Human-validated relevance score (0-1)
- **consistency_score**: float - For repeated queries, similarity of results (0-1)
- **validation_timestamp**: datetime - When validation was performed