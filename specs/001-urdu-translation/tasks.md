# Implementation Tasks: Urdu Translation for Book Chapters

**Feature**: Urdu Translation for Book Chapters
**Branch**: `001-urdu-translation`
**Spec**: [specs/001-urdu-translation/spec.md](specs/001-urdu-translation/spec.md)
**Plan**: [specs/001-urdu-translation/plan.md](specs/001-urdu-translation/plan.md)

## Phase 1: Setup and Environment

- [X] T001 Set up OpenRouter API key in backend environment configuration
- [X] T002 Install OpenAI SDK and related dependencies for Context7 MCP server integration
- [X] T003 Create translation-specific configuration settings for API integration

## Phase 2: Foundational Components

- [X] T004 Create Translation Agent using Context7 MCP server with OpenAI Agent SDK (`backend/src/services/translation_agent.py`)
- [X] T005 Define translation request/response models (`backend/src/models/translation.py`)
- [X] T006 Set up in-memory caching mechanism for translated content per user and chapter
- [X] T007 Create authentication middleware to verify user session for translation access

## Phase 3: User Story 1 - Translate Chapter to Urdu [US1]

**Goal**: Enable logged-in users to translate chapter content to Urdu with a single button press while preserving formatting and structure.

**Independent Test**: User can click "Translate to Urdu" button and see chapter content in Urdu while maintaining headings, lists, and formatting.

**Tasks**:

- [X] T008 [P] [US1] Create translation endpoint in backend (`backend/src/endpoints/translation.py`)
- [X] T009 [P] [US1] Implement translation service that calls OpenRouter API with proper system prompts
- [X] T010 [P] [US1] Add translation button to ChapterContent component (`frontend/src/components/Chapter/ChapterContent.tsx`)
- [X] T011 [US1] Implement frontend translation service to communicate with backend API (`frontend/src/services/translationService.ts`)
- [X] T012 [US1] Add state management for translation toggle in TranslationContext (`frontend/src/contexts/TranslationContext.tsx`)
- [X] T013 [US1] Implement content switching functionality (original ↔ Urdu)
- [X] T014 [US1] Add loading indicators during translation process
- [X] T015 [US1] Handle translation errors gracefully with user notifications

## Phase 4: User Story 2 - Access Translation Button [US2]

**Goal**: Display translation button at the start of each chapter only for logged-in users.

**Independent Test**: Verify the translation button appears for logged-in users at the beginning of each chapter and is hidden for non-logged-in users.

**Tasks**:

- [X] T016 [P] [US2] Integrate authentication check in ChapterContent component to show/hide translation button
- [X] T017 [US2] Style translation button to be clearly visible and accessible
- [X] T018 [US2] Position translation button at the start of each chapter
- [X] T019 [US2] Add appropriate ARIA labels for accessibility

## Phase 5: User Story 3 - Preserve Content Structure [US3]

**Goal**: Ensure translated content maintains original structure and formatting (headings, lists, code blocks).

**Independent Test**: Translate content and verify that headings, lists, code blocks, and paragraphs maintain their original structure and formatting.

**Tasks**:

- [X] T020 [P] [US3] Configure Translation Agent system prompt to preserve formatting and structure
- [X] T021 [P] [US3] Ensure Translation Agent does not translate code blocks or inline code
- [X] T022 [US3] Test preservation of markdown elements (headings, lists, emphasis)
- [X] T023 [US3] Verify that special formatting (tables, quotes, etc.) is maintained
- [X] T024 [US3] Implement proper handling of nested structures

## Phase 6: User Story 4 - Switch Between Languages [US4]

**Goal**: Allow users to toggle between original and Urdu-translated content.

**Independent Test**: User can translate content to Urdu and then switch back to original language.

**Tasks**:

- [X] T025 [P] [US4] Implement toggle functionality in frontend component
- [X] T026 [US4] Add "Original Language" button for reverting to source content
- [X] T027 [US4] Preserve scroll position when toggling between languages
- [X] T028 [US4] Maintain any user interactions (collapsible sections, etc.) across language switches

## Phase 7: Polish & Cross-Cutting Concerns

- [X] T029 Implement caching mechanism to store translations per user and chapter
- [X] T030 Add rate limiting to translation API endpoints to prevent abuse
- [X] T031 Add comprehensive error handling for API connectivity issues
- [X] T032 Optimize translation performance for large chapters
- [X] T033 Add logging for translation requests and errors
- [X] T034 Update documentation with translation feature usage
- [X] T035 Test responsive behavior of translation UI on mobile devices

## Dependencies

**User Story Completion Order**:
1. US1 (Core translation functionality) → US2 (Button visibility) → US3 (Structure preservation) → US4 (Language switching)

## Parallel Execution Examples

**Per User Story**:
- US1: Tasks T008-T015 can be developed in parallel (backend endpoints + frontend components)
- US2: Tasks T016-T019 can be developed together with frontend team
- US3: Tasks T020-T024 focus on backend agent configuration and testing
- US4: Tasks T025-T028 enhance frontend experience

## Implementation Strategy

**MVP First**: Implement US1 (core translation) with minimal UI, then enhance with additional user stories.

**Incremental Delivery**:
- Sprint 1: US1 (basic translation functionality)
- Sprint 2: US2 (button visibility) + US3 (structure preservation)
- Sprint 3: US4 (language switching) + polish tasks