# Data Model: Urdu Translation for Book Chapters

## Overview
This document defines the data structures and models for the authenticated Urdu translation feature.

## Core Entities

### TranslationRequest
Represents a user's request to translate chapter content to Urdu

**Fields**:
- `id` (string, UUID) - Unique identifier for the translation request
- `user_id` (string) - ID of the authenticated user requesting translation
- `chapter_id` (string) - Identifier of the chapter to be translated
- `target_language` (string) - Target language code (fixed as "ur" for Urdu)
- `original_content` (string) - Original chapter content to be translated
- `created_at` (datetime) - Timestamp when the request was made

**Validation rules**:
- `user_id` must be valid and authenticated
- `chapter_id` must exist in the system
- `target_language` must be "ur"
- `original_content` must not exceed API limits

### TranslationResponse
Represents the result of a translation operation

**Fields**:
- `success` (boolean) - Whether the translation was successful
- `translated_content` (string) - The translated content in Urdu
- `error_message` (string, optional) - Error message if translation failed
- `request_id` (string) - Reference to the original translation request
- `cached` (boolean) - Whether the response came from cache

### CachedTranslation
Represents a cached translation for performance optimization

**Fields**:
- `user_id` (string) - ID of the user who requested the translation
- `chapter_id` (string) - Identifier of the translated chapter
- `translated_content` (string) - The translated content in Urdu
- `created_at` (datetime) - Timestamp when the translation was cached
- `expires_at` (datetime) - Timestamp when the cache entry expires

**State transitions**:
- New translation request → Translation in progress → Cached translation stored

### TranslationAgentConfig
Configuration for the Translation Agent system prompt

**Fields**:
- `system_prompt` (string) - Fixed system prompt for professional Urdu translation
- `preserve_formatting` (boolean) - Whether to preserve headings, bullets, and formatting
- `skip_code_blocks` (boolean) - Whether to skip translating code blocks
- `skip_explanations` (boolean) - Whether to avoid adding extra explanations

**System prompt template**:
```
You are a professional Urdu translator. Translate the provided content to Urdu while following these rules:
1. Maintain professional, academic tone appropriate for technical content
2. Preserve all headings, bullet points, lists, and formatting structure
3. Do NOT translate code blocks, keep them exactly as in the original
4. Do NOT add explanations, summaries, or extra text beyond the translation
5. Maintain technical terminology accuracy
6. Ensure readability and comprehension in Urdu
```

## Relationships
- One user can have multiple translation requests (1:M)
- One chapter can be translated by multiple users (1:M)
- Each user-chapter combination can have one cached translation (1:1)

## Validation Rules from Requirements
- Translation requests must come from authenticated users
- Only Urdu (language code "ur") translations are supported
- Code blocks must not be translated
- Original content structure must be preserved
- Translations should maintain readability and comprehension
- Error handling must be graceful with appropriate user notifications