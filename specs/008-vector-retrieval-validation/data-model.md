# Data Model: Vector Retrieval Validation

## Entities

### QueryRequest
- **query_text**: string - The input query text to be converted to embedding
- **top_k**: integer - Number of results to retrieve (default: 5)
- **metadata_filter**: optional object - Additional filters for search (optional)

### QueryResponse
- **query_text**: string - The original input query text
- **results**: array of SearchResult objects - List of retrieved results
- **execution_time**: float - Time taken to execute the query in seconds
- **timestamp**: datetime - When the query was executed

### SearchResult
- **content_chunk**: string - The retrieved content chunk text
- **similarity_score**: float - Cosine similarity score (0-1)
- **metadata**: object - Full metadata from the stored chunk
  - **url**: string - Source URL of the content
  - **title**: string - Title of the source document
  - **chunk_index**: integer - Index of this chunk within the source
  - **source_metadata**: object - Additional source metadata
  - **created_at**: datetime - When the chunk was created
- **rank**: integer - Position in the results (1-indexed)

### ValidationResult
- **query_request**: QueryRequest - The original query that was validated
- **query_response**: QueryResponse - The response received
- **metadata_preserved**: boolean - Whether all metadata fields are present and valid
- **relevance_score**: float - Human-validated relevance score (0-1)
- **consistency_score**: float - For repeated queries, similarity of results (0-1)
- **validation_timestamp**: datetime - When validation was performed

### RetrievalMetrics
- **total_queries**: integer - Number of queries processed
- **successful_queries**: integer - Number of queries that returned results
- **avg_response_time**: float - Average query response time in seconds
- **avg_similarity_score**: float - Average similarity score of results
- **metadata_accuracy**: float - Percentage of results with complete metadata (0-100)
- **relevance_accuracy**: float - Percentage of results that are semantically relevant (0-100)
- **consistency_rate**: float - Percentage of consistent results across repeated queries (0-100)