# Implementation Tasks: AI-Powered Chapter Personalization

**Feature**: AI-Powered Chapter Personalization (006-personalize-content)
**Branch**: `006-personalize-content`
**Updated**: 2025-12-27
**Input**: spec.md, plan.md, data-model.md, contracts/personalization-api.yaml, research.md

## Implementation Strategy

**MVP First**: Implement User Story 1 (Trigger AI Personalization) as the minimum viable product, which includes the core backend AI agent and frontend trigger. This delivers the core value proposition with AI-powered content transformation based on reading level.

**Incremental Delivery**:
- Phase 1: Setup and configuration
- Phase 2: Backend AI Agent (Core)
- Phase 3: Frontend State & Service
- Phase 4: User Story 1 (P1) - Trigger AI Personalization
- Phase 5: User Story 2 (P2) - View Personalized Content
- Phase 6: User Story 3 (P3) - Revert to Original
- Phase 7: Polish and cross-cutting concerns

## Dependencies

**User Story Completion Order**:
1. User Story 1 (P1) - Trigger AI Personalization (base functionality)
2. User Story 2 (P2) - View Personalized Content (depends on US1)
3. User Story 3 (P3) - Revert to Original (depends on US2)

**Parallel Execution Examples**:
- [P] Tasks within each phase that operate on different files/components can be executed in parallel
- Backend AI agent can be developed while frontend context is updated
- PersonalizationPanel can be built in parallel with backend endpoint

**Key Pattern Reuse**:
| Pattern | Source File | Reuse For |
|---------|-------------|-----------|
| OpenRouter API call | `backend/src/services/translation_agent.py` | personalization_agent.py |
| Context state | `frontend/src/contexts/TranslationContext.tsx` | PersonalizationContext updates |
| Content extraction | `frontend/src/components/Chapter/ChapterContent.tsx` | Get chapter HTML |

---

## Phase 1: Setup & Configuration

### Goal
Prepare project structure and verify dependencies for AI personalization

- [X] T001 Verify OpenRouter API key is configured in `.env` (OPENROUTER_API_KEY)
- [X] T002 [P] Verify `backend/src/services/` directory exists
- [X] T003 [P] Verify frontend Personalization component directory exists
- [X] T004 [P] Create `frontend/src/components/Personalization/PersonalizationPanel.module.css`

---

## Phase 2: Backend AI Agent (Priority: P1)

### Goal
Create the AI personalization agent using OpenRouter LLM (same pattern as translation)

### Key Files
- `backend/src/services/personalization_agent.py` (CREATE)
- `backend/src/personalization_config.py` (CREATE)
- `backend/endpoints/personalization.py` (MODIFY)

- [X] T005 [P] Create `backend/src/personalization_config.py` with AI system prompts
  - Define PERSONALIZATION_SYSTEM_PROMPT with reading level guidelines
  - Define technical explanation rules
  - Define example density rules
  - Define structure preservation rules (never modify headings, code blocks)

- [X] T006 [P] Create `backend/src/services/personalization_agent.py`
  - Copy pattern from `translation_agent.py`
  - Implement `personalize_content(content, preferences, user_profile)` function
  - Use OpenRouter with mistralai/devstral-2512:free model
  - Handle reading levels: beginner, intermediate, advanced
  - Apply technical explanations toggle
  - Apply example density settings
  - Return personalized HTML preserving structure

- [X] T007 Add `/api/personalization/ai-personalize` POST endpoint
  - Location: `backend/endpoints/personalization.py`
  - Request body: chapter_id, content, preferences, user_profile
  - Response: success, personalized_content, preferences_applied, processing_time_ms
  - Require authentication (Bearer token)
  - Handle 400 (bad request), 401 (unauthorized), 503 (AI unavailable)

- [X] T008 Add `/api/personalization/health` GET endpoint
  - Check if personalization service is available
  - Check if OpenRouter is configured
  - No authentication required

- [X] T009 [P] Register new personalization endpoints in `backend/main.py`

- [ ] T010 Test AI personalization endpoint with curl
  ```bash
  curl -X POST http://localhost:8000/api/personalization/ai-personalize \
    -H "Authorization: Bearer <token>" \
    -H "Content-Type: application/json" \
    -d '{"chapter_id": "test", "content": "<p>ROS2 middleware...</p>", "preferences": {"reading_level": "beginner", "technical_explanations": true, "example_density": "normal"}}'
  ```

---

## Phase 3: Frontend State & Service (Priority: P1)

### Goal
Update frontend state management and API service for AI personalization

### Key Files
- `frontend/src/contexts/PersonalizationContext.tsx` (MODIFY)
- `frontend/src/services/personalizationService.ts` (MODIFY)

- [X] T011 Update PersonalizationContext with AI state
  - Add `preferences: { readingLevel, technicalExplanations, exampleDensity }`
  - Add `personalizedContent: Record<string, PersonalizedContentEntry>`
  - Add `isPersonalizing: boolean`
  - Add `personalizationError: string | null`
  - Add `isPanelOpen: boolean`
  - Add `activeChapterId: string | null`

- [X] T012 Add PersonalizationContext methods
  - `personalizeChapter(chapterId: string, content: string): Promise<void>`
  - `revertToOriginal(chapterId: string): void`
  - `updatePreferences(prefs: Partial<Preferences>): void`
  - `openPanel(chapterId: string): void`
  - `closePanel(): void`
  - `getContentForChapter(chapterId: string): string | null`

- [X] T013 Add default preference mapping from user profile
  ```typescript
  const mapProfileToDefaults = (profile) => ({
    readingLevel: profile.software_experience || 'intermediate',
    technicalExplanations: profile.software_experience === 'beginner',
    exampleDensity: 'normal'
  });
  ```

- [X] T014 Update personalizationService.ts with `aiPersonalize()` function
  - POST to `/api/personalization/ai-personalize`
  - Include Authorization header with JWT token
  - Handle response and errors
  - Return PersonalizationResponse type

---

## Phase 4: User Story 1 - Trigger AI Personalization (Priority: P1)

### Goal
Logged-in users can click "Personalize Content" button to open a settings panel and trigger AI-based content personalization for the current chapter.

### Independent Test Criteria
Can be tested by logging in, navigating to a chapter, clicking the personalization button, adjusting settings in the panel, clicking "Apply Personalization", and seeing the loading state.

### Key Files
- `frontend/src/components/Personalization/PersonalizationButton.tsx` (MODIFY)
- `frontend/src/components/Personalization/PersonalizationPanel.tsx` (CREATE)
- `frontend/src/components/Personalization/PersonalizationPanel.module.css` (CREATE)

- [X] T015 [US1] Create PersonalizationPanel component
  - Drawer/slide-in panel from right side
  - Reading Level selector: Beginner | Intermediate | Advanced
  - Technical Terms toggle: Show inline explanations checkbox
  - Example Density: Minimal | Normal | Detailed radio buttons
  - "Apply Personalization" button
  - Close (X) button

- [X] T016 [US1] Style PersonalizationPanel with CSS module
  - Clean, minimal design matching existing UI
  - Proper mobile responsiveness
  - Loading state styling

- [X] T017 [US1] Update PersonalizationButton for AI workflow
  - Default state: "Personalize Content" (blue, opens panel)
  - Processing state: "Personalizing..." with spinner
  - Personalized state: "Revert to Original" (green)
  - Connect to PersonalizationContext

- [X] T018 [US1] Implement panel open/close logic
  - Button click opens PersonalizationPanel
  - Panel captures current chapter ID
  - Close on X click or outside click

- [X] T019 [US1] Implement "Apply Personalization" handler
  - Extract chapter content via `document.querySelector('.chapter-content').innerHTML`
  - Call `personalizeChapter(chapterId, content)` from context
  - Show loading state during API call
  - Close panel on success

- [X] T020 [US1] Add authentication guard
  - PersonalizationButton only visible when user is authenticated
  - Redirect or show message for unauthenticated users

- [ ] T021 [US1] Test personalization trigger with acceptance scenario
  - Login as user
  - Navigate to chapter
  - Click "Personalize Content"
  - Adjust settings in panel
  - Click "Apply Personalization"
  - Verify loading state appears
  - Verify API is called with correct parameters

---

## Phase 5: User Story 2 - View Personalized Content (Priority: P2)

### Goal
After AI personalization completes, users see the transformed content with appropriate reading level adjustments while the original structure (headings, code blocks) is preserved.

### Independent Test Criteria
Can be tested by completing personalization and verifying content changes reflect the selected reading level, with all headings and code blocks unchanged.

### Key Files
- `frontend/src/components/Chapter/ChapterContent.tsx` (MODIFY)
- `frontend/src/theme/DocItem/Layout/index.tsx` (MODIFY)

- [X] T022 [US2] Update ChapterContent to display personalized content
  - Check `personalizedContent[chapterId]` in context
  - If personalized, render `content` from state
  - If not personalized, render original Docusaurus content
  - Use `dangerouslySetInnerHTML` for HTML content (same as translation)

- [X] T023 [US2] Add visual indicator for personalized state
  - Small badge or icon showing "Personalized"
  - Reading level indicator (e.g., "Beginner Mode")
  - Timestamp of personalization

- [X] T024 [US2] Integrate PersonalizationPanel in DocItem Layout
  - Add PersonalizationPanel component to layout
  - Position as slide-in drawer
  - Connect to PersonalizationContext

- [X] T025 [US2] Handle loading state during personalization
  - Show skeleton or overlay on content area
  - Disable interactions during processing
  - Show progress indicator

- [ ] T026 [US2] Test personalized content display
  - Complete personalization for beginner level
  - Verify simpler language in content
  - Verify technical terms have explanations
  - Verify headings unchanged
  - Verify code blocks unchanged

- [ ] T027 [US2] Test structure preservation
  - Personalize chapter with multiple headings
  - Verify all heading text identical to original
  - Verify all code blocks identical to original
  - Verify image references preserved

---

## Phase 6: User Story 3 - Revert to Original (Priority: P3)

### Goal
Users can instantly revert to the original chapter content at any time, with the original content restored from frontend state (under 1 second).

### Independent Test Criteria
Can be tested by personalizing content, clicking "Revert to Original", and verifying original content is restored instantly without API call.

- [X] T028 [US3] Implement revert button state
  - After personalization, button shows "Revert to Original"
  - Button styled differently (green or toggle style)

- [X] T029 [US3] Implement instant revert logic
  - Call `revertToOriginal(chapterId)` from context
  - Delete entry from `personalizedContent` state
  - ChapterContent re-renders with original content
  - No API call required (instant)

- [X] T030 [US3] Add revert confirmation (optional)
  - Consider: Direct revert or confirmation dialog
  - Spec says instant, so likely no confirmation needed

- [ ] T031 [US3] Test revert functionality
  - Personalize a chapter
  - Click "Revert to Original"
  - Verify content reverts in under 1 second
  - Verify button returns to "Personalize Content"
  - Verify no network request made

- [ ] T032 [US3] Test re-personalization after revert
  - Revert personalization
  - Click "Personalize Content" again
  - Adjust to different settings
  - Apply and verify new personalization works

---

## Phase 7: Polish & Cross-Cutting Concerns

### Goal
Complete implementation with error handling, edge cases, and performance optimization

### Error Handling

- [ ] T033 Handle AI service unavailable (503)
  - Show user-friendly error message
  - Provide retry button
  - Log error for monitoring

- [ ] T034 Handle authentication failure (401)
  - Redirect to login
  - Clear stale tokens
  - Show appropriate message

- [ ] T035 Handle timeout (>30 seconds)
  - Cancel request after timeout
  - Show error with retry option
  - Suggest trying shorter chapter

- [ ] T036 Handle malformed AI response
  - Validate response has personalized_content
  - Fallback to original if invalid
  - Log warning for debugging

### Edge Cases

- [ ] T037 Handle unauthenticated users
  - Hide PersonalizationButton completely
  - No error, just not visible

- [ ] T038 Handle long chapters (>10,000 words)
  - Consider chunking strategy if needed
  - Show progress for long operations
  - Test with actual long chapter

- [ ] T039 Handle rapid repeated clicks
  - Debounce Apply button
  - Disable during processing
  - Prevent duplicate API calls

- [ ] T040 Handle navigation during personalization
  - Cancel in-flight request
  - Clean up state properly
  - No stale content shown

- [ ] T041 Handle page refresh
  - Session-only storage means content is lost
  - User sees original content (expected behavior)
  - Button returns to default state

### Performance

- [ ] T042 Ensure personalization response < 10 seconds
  - Monitor processing_time_ms in response
  - Optimize prompt if too slow
  - Consider streaming for long content

- [ ] T043 Ensure revert response < 1 second
  - Already instant from state
  - Verify no unnecessary re-renders

### Testing

- [ ] T044 Test all three reading levels
  - Beginner: simpler language, explanations
  - Intermediate: balanced
  - Advanced: concise, professional

- [ ] T045 Test technical explanations toggle
  - ON: Terms have parenthetical explanations
  - OFF: Original terms unchanged

- [ ] T046 Test example density settings
  - Minimal: fewer examples
  - Normal: original examples
  - Detailed: more examples

- [ ] T047 Test with RTL content (if applicable)
  - Verify personalized content preserves RTL

- [ ] T048 Integration test: full personalization workflow
  - Login → Navigate → Personalize → View → Revert

### Documentation

- [ ] T049 Update API documentation with new endpoint
  - Document /ai-personalize request/response
  - Include example curl commands
  - Note authentication requirements

- [ ] T050 Add user guide for personalization feature
  - How to personalize content
  - Explanation of reading levels
  - How to revert

---

## Acceptance Criteria Summary

| Criteria | Task Reference |
|----------|---------------|
| Button visible only to logged-in users | T020, T037 |
| Single-action per chapter trigger | T019 |
| Preserves headings, code blocks | T026, T027 |
| Response < 10 seconds | T042 |
| Revert < 1 second | T043 |
| Session-only storage | T011 (context state) |
| Reading level adjustment | T005, T006, T044 |
| Technical explanations toggle | T005, T045 |
| Example density control | T005, T046 |

---

## Task Completion Tracking

**Total Tasks**: 50
**Completed**: 24
**In Progress**: 0
**Remaining**: 26

**By Priority**:
- P1 (Core): T001-T021 (21 tasks) - 20 complete, 1 testing remaining
- P2 (UX): T022-T027 (6 tasks) - 4 complete, 2 testing remaining
- P3 (Revert): T028-T032 (5 tasks) - 3 complete, 2 testing remaining
- Polish: T033-T050 (18 tasks) - 0 complete
