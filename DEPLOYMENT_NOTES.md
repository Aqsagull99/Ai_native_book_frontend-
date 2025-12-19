# Deployment Notes for Production

## Backend Environment Variables Required on Render

When deploying the backend to Render, ensure the following environment variables are set:

### Required Variables
- `DATABASE_URL` - PostgreSQL database URL for production
- `SECRET_KEY` - Strong, random secret key for authentication (32+ characters)
- `PORT` - Port number (Render typically provides this, default is usually 10000+)

### API Keys
- `GEMINI_API_KEY` - Google Gemini API key for LLM functionality
- `COHERE_API_KEY` - Cohere API key for content embedding

### Vector Database Configuration
- `QDRANT_URL` - Qdrant cloud instance URL
- `QDRANT_API_KEY` - Qdrant cloud instance API key
- `QDRANT_COLLECTION_NAME` - Collection name (default: "ai_native_book")

### Content URLs
- `BOOK_BASE_URL` - Base URL of the book site (e.g., "https://ai-native-book-frontend.vercel.app")
- `TARGET_SITE` - Target site URL (same as BOOK_BASE_URL)
- `SITEMAP_URL` - Sitemap URL (e.g., "https://ai-native-book-frontend.vercel.app/sitemap.xml")

### Model Configuration
- `GEMINI_MODEL_NAME` - Gemini model name (e.g., "gemini-1.5-flash" or "gemini-pro")

## Frontend Configuration

The frontend should have the following environment variable:
- `REACT_APP_API_BASE_URL` - Should be set to your backend URL (e.g., "https://your-backend.onrender.com")

This is already configured in the frontend `.env` file.

## Common Issues and Troubleshooting

1. **JSON Parsing Error**: "Failed to execute 'json' on 'Response': Unexpected end of JSON input"
   - This typically occurs when the backend returns HTML or plain text instead of JSON
   - Ensure all error handlers return proper JSON responses
   - Check that CORS is properly configured to allow the frontend domain

2. **CORS Errors**:
   - Make sure the frontend domain is included in the CORS allow_origins list
   - Check that credentials are properly handled

3. **API Key Issues**:
   - Verify all required API keys are set in the backend environment
   - Check that the API keys have the necessary permissions

## Health Check Endpoints

- Backend health: `GET /health`
- RAG agent health: `GET /api/rag/health`

## Build and Deployment Commands

### Backend (Render)
```bash
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port $PORT
```

### Frontend (Vercel/Netlify)
```bash
npm install
npm run build
```