# Production Checklist for Backend Deployment

## Required Environment Variables on Render

### Essential Variables
- `PORT` - Render provides this automatically
- `SECRET_KEY` - Strong secret key for JWT tokens (32+ characters)
- `DATABASE_URL` - PostgreSQL database URL

### RAG Service Variables (Optional but needed for full functionality)
- `GEMINI_API_KEY` - Google Gemini API key
- `QDRANT_URL` - Qdrant vector database URL
- `QDRANT_API_KEY` - Qdrant database API key
- `GEMINI_MODEL_NAME` - Model name (default: gemini-1.5-flash)

### Content Embedding Variables (Optional)
- `COHERE_API_KEY` - Cohere API key
- `BOOK_BASE_URL` - Base URL of the book site
- `TARGET_SITE` - Target site URL
- `SITEMAP_URL` - Sitemap URL

## Deployment Commands for Render

### Build Command
```bash
pip install -r requirements.txt
```

### Start Command
```bash
uvicorn main:app --host 0.0.0.0 --port $PORT
```

## Health Check Endpoints

### Main Health Check
- `GET /health` - Returns system health status

### RAG Service Health Check
- `GET /api/rag/health` - Returns RAG agent health status

## Authentication Endpoints

### Signup
- `POST /api/auth/signup` - Register new user

### Signin
- `POST /api/auth/signin` - Authenticate user

### Profile Management
- `GET /api/auth/profile` - Get user profile
- `PUT /api/auth/profile` - Update user profile

## RAG Agent Endpoints

### Query Processing
- `POST /api/rag/query` - Process user queries
- `GET /api/rag/health` - Health check for RAG service

## Troubleshooting Steps

### 1. Check Backend Health
```bash
curl https://your-backend-url.onrender.com/health
```

### 2. Check Authentication Endpoints
```bash
curl -X POST https://your-backend-url.onrender.com/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123", "software_experience": "beginner", "hardware_experience": "none"}'
```

### 3. Check RAG Service (if environment variables are set)
```bash
curl -X POST https://your-backend-url.onrender.com/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{"query_text": "test query", "top_k": 5}'
```

## Common Issues and Solutions

### Issue: HTTP 405 (Method Not Allowed)
- **Cause**: Wrong HTTP method being used
- **Solution**: Ensure POST is used for signup/signin, not GET

### Issue: "Failed to fetch"
- **Cause**: CORS issues or backend not accessible
- **Solution**: Check CORS configuration and network connectivity

### Issue: Backend not starting
- **Cause**: Missing required environment variables
- **Solution**: Ensure at least SECRET_KEY and DATABASE_URL are set

### Issue: RAG service unavailable
- **Cause**: Missing GEMINI_API_KEY, QDRANT_URL, or QDRANT_API_KEY
- **Solution**: Set the required environment variables or accept degraded functionality