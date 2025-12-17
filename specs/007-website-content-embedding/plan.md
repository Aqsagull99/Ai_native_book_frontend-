# Implementation Plan: Website Content Embedding & Vector Storage Pipeline

**Branch**: `007-website-content-embedding` | **Date**: 2025-12-15 | **Spec**: [specs/007-website-content-embedding/spec.md](file:///home/aqsagulllinux/Robot_Book_Hackathon/specs/007-website-content-embedding/spec.md)
**Input**: Feature specification from `/specs/007-website-content-embedding/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a website content embedding pipeline that crawls deployed Docusaurus book pages, extracts text content, chunks it consistently, generates vector embeddings using Cohere, and stores them in Qdrant Cloud for RAG applications. This enables semantic retrieval for a RAG chatbot by converting website content into vector embeddings.

Core functions to be implemented: `get_all_url` for crawling, `extract_text_from_url` for content extraction, and `chunk_text` for text segmentation, with Qdrant collection "ai_native_book" for storage and a main execution function to orchestrate the entire pipeline.



## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.12
**Primary Dependencies**: httpx, beautifulsoup4, cohere, qdrant-client, python-dotenv, langchain-text-splitters
**Storage**: Qdrant Cloud (vector database), with temporary local processing
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server for backend processing
**Project Type**: backend service for content processing
**Performance Goals**: Process all public book pages within 45 minutes with 99%+ success rate
**Constraints**: Must respect website rate limits, handle content updates, and preserve document metadata
**Scale/Scope**: Handle all public book pages from the deployed Docusaurus site

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] All dependencies are properly licensed and documented
- [x] Error handling and logging are implemented for all operations
- [x] Rate limiting is respected to avoid overloading the website
- [x] Security considerations for API keys and sensitive data are addressed
- [x] Performance requirements align with the 45-minute processing goal

## Project Structure

### Documentation (this feature)

```text
specs/007-website-content-embedding/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── content_embedding/
│   │   ├── __init__.py
│   │   ├── crawler.py          # Contains get_all_url function
│   │   ├── text_extractor.py   # Contains extract_text_from_url function
│   │   ├── chunker.py          # Contains chunk_text function
│   │   ├── qdrant_service.py   # Contains Qdrant collection creation and chunk saving
│   │   ├── embedding_service.py # Main service orchestrating the pipeline
│   │   └── utils.py            # Utility functions
│   ├── models/
│   ├── services/
│   └── api/
└── tests/
    ├── content_embedding/
    │   ├── test_crawler.py
    │   ├── test_text_extractor.py
    │   ├── test_chunker.py
    │   ├── test_qdrant_service.py
    │   └── test_embedding_service.py
    ├── contract/
    ├── integration/
    └── unit/
```

**Structure Decision**: Backend service structure with dedicated content embedding module containing the core functions for crawling, text extraction, chunking, and Qdrant integration. The functions get_all_url, extract_text_from_url, and chunk_text are implemented in separate modules for maintainability and testability, with Qdrant service for creating collection "ai_native_book" and saving chunks.

## Core Functions Implementation

### 1. get_all_url function
- **Purpose**: Crawl the deployed Docusaurus book and extract all accessible URLs
- **Location**: `backend/src/content_embedding/crawler.py`
- **Dependencies**: httpx, beautifulsoup4, sitemap parsing
- **Function signature**: `async def get_all_url(base_url: str, max_depth: int = 2) -> List[str]`
- **Features**:
  - Sitemap parsing to discover URLs
  - Recursive crawling with depth limit
  - Rate limiting to respect website policies
  - Error handling for inaccessible URLs

### 2. extract_text_from_url function
- **Purpose**: Extract clean text content from a single web page
- **Location**: `backend/src/content_embedding/text_extractor.py`
- **Dependencies**: httpx, beautifulsoup4
- **Function signature**: `async def extract_text_from_url(url: str) -> Dict[str, Any]`
- **Features**:
  - HTML parsing and cleaning
  - Extraction of title and main content
  - Preservation of document structure and metadata
  - Handling of Docusaurus-specific selectors

### 3. chunk_text function
- **Purpose**: Split extracted text into manageable chunks for embedding
- **Location**: `backend/src/content_embedding/chunker.py`
- **Dependencies**: langchain-text-splitters
- **Function signature**: `def chunk_text(text: str, chunk_size: int = 512, overlap: int = 102) -> List[Dict[str, Any]]`
- **Features**:
  - Configurable chunk size (default 512 tokens)
  - Configurable overlap (default 20% of chunk size)
  - Metadata preservation for each chunk
  - Context-aware splitting to maintain semantic boundaries

### 4. Qdrant Collection and Chunk Storage
- **Purpose**: Create Qdrant collection "ai_native_book" and save text chunks as vector embeddings
- **Location**: `backend/src/content_embedding/qdrant_service.py`
- **Dependencies**: qdrant-client, cohere
- **Function signatures**:
  - `async def create_collection(collection_name: str = "ai_native_book") -> bool`
  - `async def save_chunks_to_qdrant(chunks: List[Dict], collection_name: str = "ai_native_book") -> bool`
- **Features**:
  - Creates Qdrant collection with appropriate vector dimensions for Cohere embeddings
  - Generates embeddings using Cohere API
  - Stores chunks with metadata in Qdrant Cloud
  - Implements batch processing for efficiency

### 5. Main Execution Function
- **Purpose**: Orchestrate the entire pipeline from URL crawling to Qdrant storage
- **Location**: `backend/src/content_embedding/embedding_service.py`
- **Function signature**: `async def main() -> None` (executable main function)
- **Features**:
  - Executes the complete pipeline: get_all_url → extract_text_from_url → chunk_text → save to Qdrant
  - Implements proper error handling and logging
  - Provides progress tracking and status updates

## Implementation Phases

### Phase 1: Core Function Implementation
1. Implement `get_all_url` function with sitemap parsing and crawling
2. Implement `extract_text_from_url` function with content extraction
3. Implement `chunk_text` function with intelligent text splitting
4. Write unit tests for each function

### Phase 2: Qdrant Integration
1. Implement Qdrant collection creation with name "ai_native_book"
2. Implement chunk storage to Qdrant with Cohere embeddings
3. Add proper error handling and retry logic for Qdrant operations

### Phase 3: Pipeline Orchestration and Execution
1. Create main execution function that orchestrates the entire pipeline
2. Implement the full workflow: crawling → extraction → chunking → embedding → storage
3. Add comprehensive error handling, logging, and progress tracking
4. Execute the main function to process the website content

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [All requirements met] |
