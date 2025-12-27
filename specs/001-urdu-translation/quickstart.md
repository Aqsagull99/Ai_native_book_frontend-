# Quickstart Guide: Urdu Translation for Book Chapters

## Overview
This guide provides the essential information to set up and run the authenticated Urdu translation feature.

## Prerequisites
- Python 3.12 with pip
- Node.js 18+ and npm/yarn
- OpenRouter API key
- Existing authentication system (users must be logged in)

## Environment Configuration
Create/update `.env` file in the backend directory:

```bash
# OpenRouter API configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here

# Database configuration (existing)
DATABASE_URL=postgresql://username:password@localhost/dbname

# Other existing environment variables
JWT_SECRET=your_jwt_secret
```

## Installation

### Backend Setup
```bash
# Navigate to backend directory
cd backend

# Install dependencies
pip install openai fastapi pydantic python-multipart

# Or if using a requirements.txt file
pip install -r requirements.txt
```

### Frontend Setup
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install
# or
yarn install
```

## Core Files Overview

### Backend
- `src/services/translation_agent.py` - Main translation agent implementation
- `src/endpoints/translation.py` - Translation API endpoints
- `src/models/translation.py` - Data models for translation requests/responses

### Frontend
- `src/components/Chapter/ChapterContent.tsx` - Chapter component with translation button
- `src/services/translationService.ts` - Service for calling translation API
- `src/contexts/TranslationContext.tsx` - State management for translations

## Running the Service

### Start Backend
```bash
# From backend directory
uvicorn main:app --reload --port 8000
```

### Start Frontend
```bash
# From frontend directory
npm run start
# or
yarn start
```

## API Usage

### Translation Endpoint
```
POST /api/translation/translate
Headers:
  Authorization: Bearer <user_jwt_token>
  Content-Type: application/json

Body:
{
  "chapter_id": "chapter-identifier",
  "target_language": "ur",
  "content": "Chapter content to translate"
}
```

### Response
```json
{
  "success": true,
  "translated_content": "مترجمہ مواد یہاں ہوگا",
  "cached": false
}
```

## Key Features

### 1. Authentication Check
- Only authenticated users can access translation feature
- System verifies JWT token before processing requests

### 2. Content Preservation
- Headings, lists, and formatting preserved
- Code blocks remain untranslated
- Structure maintained during translation

### 3. Caching
- Per-user, per-chapter caching implemented
- Reduces API calls and improves response time

### 4. Error Handling
- Graceful handling of API errors
- User-friendly error messages

## Testing

### Backend Tests
```bash
# Run backend tests
pytest tests/translation_tests.py
```

### Frontend Tests
```bash
# Run frontend tests
npm test
# or
yarn test
```

## Troubleshooting

### Common Issues
1. **API Key Issues**: Verify OPENROUTER_API_KEY is set correctly
2. **Authentication Failures**: Check JWT token validity and format
3. **Rate Limits**: Monitor API usage and implement backoff strategies

### Logging
- Backend logs available in console/output
- Translation errors logged with request IDs for debugging