# Urdu Translation Feature Documentation

## Overview
The Urdu Translation feature enables authenticated users to translate book chapter content to Urdu with a single button click while preserving formatting and structure. The feature includes caching, rate limiting, and a user-friendly interface.

## Architecture

### Backend Components

#### 1. Translation Agent (`backend/src/services/translation_agent.py`)
- Uses OpenRouter API via Context7 MCP server
- Implements system prompts to preserve formatting and structure
- Handles code blocks and special formatting preservation

#### 2. Translation Service (`backend/src/services/translation_service.py`)
- Orchestrates translation functionality
- Integrates caching, API calls, and result management
- Optimized for performance with large chapters

#### 3. Translation Cache (`backend/src/services/translation_cache.py`)
- In-memory caching per user and chapter
- Configurable TTL and max size
- Automatic cleanup of expired entries

#### 4. Translation Endpoints (`backend/src/endpoints/translation.py`)
- `/api/translation/translate` - Synchronous translation
- `/api/translation/translate-async` - Asynchronous translation for large content
- `/api/translation/status` - Check async translation status
- `/api/translation/cache/check` - Check if translation is cached
- `/api/translation/health` - Health check endpoint

#### 5. Middleware
- `backend/src/middleware/translation_auth.py` - Authentication verification
- `backend/src/middleware/rate_limiter.py` - Rate limiting to prevent abuse

### Frontend Components

#### 1. Translation Context (`frontend/src/contexts/TranslationContext.tsx`)
- State management for translation status
- Loading and error handling
- Integration with translation service

#### 2. Translation Service (`frontend/src/services/translationService.ts`)
- API communication with backend
- Error handling and response processing

#### 3. Chapter Content Component (`frontend/src/components/Chapter/ChapterContent.tsx`)
- Translation button UI
- Integration with authentication and translation context
- Scroll position preservation
- Loading indicators and error messages

## Configuration

### Environment Variables
- `OPENROUTER_API_KEY` - API key for OpenRouter service
- `OPENROUTER_BASE_URL` - Base URL for OpenRouter API
- `OPENROUTER_MODEL` - Model to use for translation
- `TRANSLATION_CACHE_TTL` - Cache TTL in seconds (default: 3600)
- `TRANSLATION_MAX_CACHE_SIZE` - Maximum cache size (default: 1000)
- `TRANSLATION_RATE_LIMIT_REQUESTS` - Rate limit requests per window (default: 10)
- `TRANSLATION_RATE_LIMIT_WINDOW` - Rate limit window in seconds (default: 60)
- `TRANSLATION_TIMEOUT` - Translation timeout in seconds (default: 30)

### Configuration File
- `backend/src/translation_config.py` - Centralized configuration

## API Endpoints

### POST /api/translation/translate
Translate content synchronously.

**Request:**
```json
{
  "content": "Content to translate",
  "chapter_id": "chapter-identifier",
  "preserve_formatting": true
}
```

**Response:**
```json
{
  "translated_content": "Translated content in Urdu",
  "chapter_id": "chapter-identifier",
  "user_id": "user-id",
  "translation_id": "translation-identifier",
  "created_at": "2025-12-27T10:00:00Z"
}
```

### POST /api/translation/translate-async
Translate content asynchronously for large chapters.

**Request:**
```json
{
  "content": "Large content to translate",
  "chapter_id": "chapter-identifier",
  "preserve_formatting": true
}
```

**Response:**
```json
{
  "translation_id": "translation-identifier",
  "status": "pending",
  "chapter_id": "chapter-identifier",
  "user_id": "user-id"
}
```

### POST /api/translation/status
Check status of async translation.

**Request:**
```json
{
  "translation_id": "translation-identifier"
}
```

**Response:**
```json
{
  "translation_id": "translation-identifier",
  "status": "completed",
  "translated_content": "Translated content",
  "error_message": null
}
```

## Usage

### For Developers
1. Ensure OpenRouter API key is configured in environment
2. The feature is automatically integrated into chapter content components
3. Rate limiting is applied automatically
4. Translations are cached automatically

### For Users
1. Log in to access translation feature
2. Click "Translate to Urdu" button at the start of any chapter
3. Wait for translation to complete
4. Use "Revert to Original" to switch back to original language
5. Translations are cached for faster access on subsequent visits

## Security & Performance

### Security Features
- Authentication required for all translation endpoints
- Rate limiting to prevent API abuse
- Input validation and sanitization
- Secure API key handling

### Performance Optimizations
- In-memory caching of translations
- Asynchronous processing for large content
- Content length optimization
- Efficient data structures for cache management

## Error Handling
- Graceful degradation when translation service is unavailable
- User-friendly error messages
- Automatic retry mechanisms
- Comprehensive logging for debugging

## Testing
The feature includes comprehensive testing for:
- Authentication flow
- Translation accuracy
- Caching behavior
- Rate limiting
- Error conditions
- UI interactions