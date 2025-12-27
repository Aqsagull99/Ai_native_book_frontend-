# Gemini 2.5 Flash Integration with Rate Limiting - Implementation Plan

## Feature Overview
Implement the gemini-2.5-flash model with rate limiting to 200 requests per day to address the current error: "Sorry, I encountered an error while processing your query." This will replace the current gemini-flash-lite-latest model and add proper rate limiting to prevent API quota issues.

## Technical Architecture


### Components to be Modified
1. **Configuration** (`backend/src/rag_agent/config.py`): Update model name and add rate limiting settings
2. **API Service** (`backend/src/rag_agent/api_service.py`): Add rate limiting middleware and request tracking
3. **Agent Implementation** (`backend/src/rag_agent/agent.py`): Ensure compatibility with gemini-2.5-flash
4. **Environment Variables**: Add rate limiting configuration

### New Components to be Created
1. **Rate Limiter Service**: Track daily request counts and enforce limits
2. **Rate Limit Status Endpoint**: Allow monitoring of current rate limit status

## Implementation Details

### Model Configuration
- Replace `gemini-flash-lite-latest` with `gemini-2.5-flash`
- The gemini-2.5-flash model offers improved performance and capabilities
- Ensure proper API endpoint compatibility

### Rate Limiting Strategy
- Implement a daily counter to track requests per day (reset at midnight UTC)
- Store request counts in memory with persistence to prevent loss on restart
- Add middleware to check rate limits before processing requests
- Return appropriate HTTP status code (429) when limit is exceeded
- Provide clear error messages when rate limit is reached

### Data Flow
1. User sends query to RAG agent API
2. Rate limiter checks if daily limit has been reached
3. If under limit, process query with gemini-2.5-flash model
4. If over limit, return 429 Too Many Requests with descriptive message
5. Update daily request counter after each request

## File Structure
```
backend/
├── src/
│   └── rag_agent/
│       ├── config.py              # Updated with rate limiting settings
│       ├── api_service.py         # Updated with rate limiting middleware
│       ├── agent.py               # Updated for gemini-2.5-flash compatibility
│       ├── rate_limiter.py        # NEW: Rate limiting service
│       └── models.py              # Potentially updated for rate limit responses
└── main.py                        # Updated to initialize rate limiter
```

## Environment Variables
- `GEMINI_MODEL_NAME`: Set to `gemini-2.5-flash`
- `DAILY_REQUEST_LIMIT`: Set to `200` (default: 200)
- `RATE_LIMIT_WINDOW_HOURS`: Set to `24` (default: 24)

## Error Handling
- Return HTTP 429 (Too Many Requests) when rate limit is exceeded
- Include remaining time until reset in response headers
- Provide user-friendly error messages

## Testing Strategy
- Unit tests for rate limiting logic
- Integration tests for API endpoints with rate limiting
- End-to-end tests to verify rate limit behavior
- Load testing to validate rate limiting effectiveness