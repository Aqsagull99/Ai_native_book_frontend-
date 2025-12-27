# Research: Urdu Translation for Book Chapters

## Overview
This research document covers the technical investigation for implementing authenticated Urdu translation for book chapters using OpenRouter API with a dedicated Translation Agent.

## Decision: Translation Agent Implementation
**Rationale**: Based on the requirements, we need a dedicated Translation Agent that follows specific system prompts for professional Urdu translation while preserving formatting and structure. The agent will be implemented as a single file `translation_agent.py` using Context7 MCP server with OpenAI Agent SDK.

**Alternatives considered**:
1. Direct API calls to OpenRouter without an agent wrapper - Rejected because it doesn't provide the structured approach needed for consistent translation quality
2. Using a different translation service (Google Translate API, Azure Translator) - Rejected because OpenRouter was specifically requested and offers access to advanced models like Claude/GPT
3. Client-side translation - Rejected because it would expose API keys and not provide the server-side caching needed

## Decision: Authentication and Authorization
**Rationale**: Only authenticated users should access the translation feature to prevent abuse and API costs. We'll leverage the existing authentication system to verify user sessions before allowing translation requests.

**Alternatives considered**:
1. Public translation access with rate limiting - Rejected because it doesn't align with the requirement that "Translation button is visible only to logged-in users"
2. IP-based rate limiting only - Rejected because it doesn't provide user-specific tracking and caching

## Decision: Caching Strategy
**Rationale**: To optimize performance and reduce API costs, we'll implement per-user, per-chapter caching of translations. This means if a user translates a chapter to Urdu, subsequent requests for the same user and chapter will return the cached translation.

**Alternatives considered**:
1. Global caching (shared across all users) - Rejected because it doesn't account for user-specific preferences and could lead to privacy concerns
2. No caching - Rejected because it would result in unnecessary API calls and slower performance

## Decision: Content Preservation
**Rationale**: The system must preserve all markdown formatting, headings, lists, and code blocks while only translating textual content. The Translation Agent will use specific system prompts to ensure code blocks are not translated.

**Alternatives considered**:
1. Plain text translation - Rejected because it would lose all formatting and structure
2. Manual parsing and reconstruction - Rejected because it's more complex and error-prone than using an intelligent agent

## Decision: Frontend Implementation
**Rationale**: The translation button will be integrated into the existing ChapterContent component with state management to toggle between original and translated content. This provides a seamless user experience without disrupting the existing UI flow.

**Alternatives considered**:
1. Separate translation page - Rejected because it would require navigation away from the content
2. Modal overlay for translation - Rejected because inline toggle provides better user experience

## Technical Dependencies Researched

### OpenRouter API
- Provides access to advanced models (Claude, GPT) suitable for high-quality Urdu translation
- Supports structured prompting to preserve formatting
- Offers competitive pricing compared to direct provider APIs
- Well-documented API with good reliability

### Context7 MCP Server
- Allows integration of OpenAI Agent SDK as requested
- Provides structured way to implement the translation agent
- Compatible with existing Python backend infrastructure

### Docusaurus Integration
- Supports custom React components for translation functionality
- Provides hooks for content manipulation without disrupting core functionality
- Well-suited for documentation-heavy content like book chapters

## Implementation Risks and Mitigation

### API Cost Management
- **Risk**: High volume of translation requests could incur significant costs
- **Mitigation**: Implement aggressive caching, rate limiting, and monitoring of API usage

### Translation Quality
- **Risk**: AI translations may not maintain technical accuracy
- **Mitigation**: Carefully crafted system prompts that emphasize preserving technical terminology and academic accuracy

### Performance Impact
- **Risk**: Translation requests may slow down user experience
- **Mitigation**: Asynchronous processing with loading indicators, plus caching of previously translated content