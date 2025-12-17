# Research: Frontend–Backend Integration for RAG Chatbot

## Decision: CORS Configuration Approach
**Rationale**: FastAPI's `CORSMiddleware` is the standard and most secure approach for handling cross-origin requests. It allows fine-grained control over allowed origins, methods, and headers.

**Alternatives considered**:
- Proxy-based approach: More complex setup, requires additional configuration
- JSONP: Outdated approach, security concerns
- Manual headers: Less secure, doesn't follow best practices

## Decision: API Endpoint Structure
**Rationale**: Following RESTful principles with a dedicated `/query` endpoint under the RAG agent namespace provides clear separation of concerns and follows FastAPI best practices.

**Alternatives considered**:
- GraphQL: More complex for simple query-response pattern
- WebSocket: Overkill for basic query-response, adds complexity
- Server-sent events: Good for streaming but not needed for basic functionality

## Decision: Frontend Integration Pattern
**Rationale**: A dedicated React chat component that's always visible on Docusaurus pages provides the best user experience with seamless integration into the book content, allowing users to ask questions without leaving the content context.

**Alternatives considered**:
- Contextual popup that appears when user selects text: Would interrupt reading flow
- Floating button that expands into chat: Less discoverable, could be missed
- Toolbar button that opens chat in sidebar: Less integrated with content

## Decision: Response Display Format
**Rationale**: Displaying responses with source citations and links to original content is essential for an educational context, providing transparency and allowing users to verify information and explore the original sources.

**Alternatives considered**:
- Plain text response only: Doesn't meet educational transparency requirements
- Source attribution without direct links: Less convenient for users to access sources
- Confidence scores without source links: Doesn't provide source transparency

## Decision: Error Handling Strategy
**Rationale**: Showing user-friendly error messages with option to retry maintains user trust and provides actionable feedback when the RAG service is unavailable, which is critical for maintaining a positive user experience.

**Alternatives considered**:
- Show technical error details: Could confuse non-technical users
- Hide all errors: Creates uncertainty about what went wrong
- Automatic retry without user notification: Could cause unexpected behavior

## Decision: Query Length Limits
**Rationale**: Implementing reasonable query length limits (500-1000 characters) with clear user feedback balances functionality with system performance while providing clear expectations to users, preventing API overload.

**Alternatives considered**:
- No explicit limits: Could lead to API overload and poor performance
- Very restrictive limits: Would limit user expression and utility
- Very generous limits: Could impact system performance and response quality

## Decision: Selected Text Integration
**Rationale**: Including optional selected text as context in the query enhances the RAG system's ability to provide relevant responses based on user's current reading context, improving the relevance of responses.

**Alternatives considered**:
- Only use selected text to pre-populate query field: Doesn't leverage context for better responses
- Ignore selected text: Loses valuable contextual information
- Use selected text as primary query: Would override user's actual question