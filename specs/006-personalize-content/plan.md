# Implementation Plan: Authenticated Chapter Personalization (AI-Powered)

**Branch**: `006-personalize-content` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification with UX-driven AI personalization requirements
**Updated**: Transformed from gamification to AI-powered content personalization

## Summary

Transform the existing gamification-based personalization system into an AI-powered content adaptation system. Users can personalize chapter content based on their reading level (Beginner/Intermediate/Advanced) with AI adjusting explanations, examples, and technical term visibility. The system uses OpenRouter LLM API (existing integration) for intelligent content modification while preserving original structure, headings, and formatting.

**Key Changes from Previous Plan**:
- Removed bonus points focus
- Added AI-powered content personalization via LLM
- Added reading level controls (Beginner/Intermediate/Advanced)
- Added technical term explanations toggle
- Added example density control (Minimal/Normal/Detailed)
- Added session-only storage (no permanent saving)
- Added revert to original functionality

## Technical Context

**Language/Version**: Python 3.12 (backend), TypeScript (frontend)
**Primary Dependencies**: FastAPI, React, OpenRouter API (existing from translation), Docusaurus
**Storage**: Session storage for personalized content (frontend state only - no DB persistence)
**Testing**: pytest for backend, manual testing for frontend
**Target Platform**: Web application (Linux server backend, Vercel frontend)
**Project Type**: Web (frontend + backend)
**Performance Goals**: Personalization response < 10 seconds, Revert < 1 second
**Constraints**: No permanent storage of personalized content, preserve original structure 100%
**Scale/Scope**: Per-chapter personalization, session-only storage, chapters up to 10,000 words

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Accuracy through primary source verification | ✅ PASS | AI preserves original content, only adjusts presentation |
| Clarity for academic audience | ✅ PASS | Reading levels support Grade 10-12 content adaptation |
| Reproducibility | ✅ PASS | Uses existing OpenRouter integration pattern from translation |
| Traceability | ✅ PASS | AI grounded in original chapter content - no fabrication |
| Development Workflow | ✅ PASS | Uses Docusaurus, FastAPI, React per constitution |

## Project Structure

### Documentation (this feature)

```text
specs/006-personalize-content/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── personalization-api.yaml
├── checklists/          # Validation checklists
│   └── requirements.md
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── services/
│   │   └── personalization_agent.py  # NEW: AI personalization agent
│   └── personalization_config.py     # NEW: AI prompts config
├── endpoints/
│   └── personalization.py            # MODIFY: Add AI endpoint
└── services/
    └── personalization_service.py    # MODIFY: Add AI methods

frontend/
├── src/
│   ├── components/
│   │   ├── Personalization/
│   │   │   ├── PersonalizationButton.tsx      # MODIFY: AI workflow
│   │   │   ├── PersonalizationPanel.tsx       # NEW: Settings drawer
│   │   │   └── PersonalizationPanel.module.css # NEW: Panel styles
│   │   └── Chapter/
│   │       └── ChapterContent.tsx             # MODIFY: AI content display
│   ├── contexts/
│   │   └── PersonalizationContext.tsx         # MODIFY: AI state management
│   ├── services/
│   │   └── personalizationService.ts          # MODIFY: AI endpoint calls
│   └── theme/
│       └── DocItem/Layout/index.tsx           # MODIFY: Panel integration
```

**Structure Decision**: Extend existing web application structure with new AI personalization layer. Reuse OpenRouter pattern from translation feature.

## Architecture Overview

### Reading Level Personalization Matrix

| Level | Language | Technical Terms | Examples | Default For |
|-------|----------|-----------------|----------|-------------|
| **Beginner** | Simpler, more explanations | Inline explanations ON by default | Detailed (more examples) | software_exp = beginner |
| **Intermediate** | Balanced | Contextual hints | Normal amount | software_exp = intermediate |
| **Advanced** | Concise/Professional | No explanations by default | Minimal (essential only) | software_exp = advanced |

### Technical Term Explanations

- **Enabled**: AI adds inline parenthetical explanations for technical terms
  - Example: "ROS2 (Robot Operating System 2, a middleware for robot software)"
- **Disabled**: Original terms shown without modification
- Toggle controlled per reading level but user can override

### Example Density Control

| Mode | Behavior |
|------|----------|
| **Minimal** | Show only essential examples, remove supplementary ones |
| **Normal** | Show standard examples (original content) |
| **Detailed** | Add additional examples and explanations |

### Data Flow

```
User Profile (software_exp, hardware_exp)
    ↓
PersonalizationPanel (reading level, term explanations, example density)
    ↓
"Personalize Chapter" Button Click
    ↓
Frontend extracts chapter innerHTML (same pattern as translation)
    ↓
POST /api/personalization/ai-personalize
    { chapter_id, content, preferences, user_profile }
    ↓
Backend AI Agent (OpenRouter LLM - mistralai/devstral-2512:free)
    ↓
LLM applies reading level transformations:
    - Adjusts language complexity
    - Adds/removes technical explanations
    - Modifies example density
    - PRESERVES all headings, structure, code blocks
    ↓
Returns: { personalized_content, metadata }
    ↓
Frontend stores in React state (NOT database - session only)
    ↓
User sees personalized content with RTL support if needed
    ↓
"Revert to Original" button restores original instantly
```

## Phase 0: Research Decisions

### Decision 1: AI Model for Personalization
- **Decision**: Use OpenRouter with existing mistralai/devstral-2512:free model
- **Rationale**: Already integrated and tested in translation feature, free tier available, consistent architecture
- **Alternatives Rejected**:
  - Google Gemini (different pattern, would add complexity)
  - Claude API (cost concerns for free tier)

### Decision 2: Personalization Storage
- **Decision**: Session-only storage in frontend React state (not database)
- **Rationale**: Per spec requirement "No permanent saving of personalized content"
- **Implementation**: Use PersonalizationContext state, cleared on page refresh/logout
- **Alternatives Rejected**: Database storage (violates spec constraint)

### Decision 3: User Preferences Source
- **Decision**: Combined approach - Profile defaults + click-time customization panel
- **Rationale**: Best UX - user sees their defaults but can adjust before personalizing
- **Implementation**:
  1. Load defaults from user profile (software_experience, hardware_experience)
  2. Show PersonalizationPanel drawer with pre-filled values
  3. User can adjust before clicking "Apply Personalization"
- **Alternatives Rejected**:
  - Profile-only (no flexibility for per-chapter adjustments)
  - Ask every time without defaults (slow, repetitive)

### Decision 4: Content Extraction Method
- **Decision**: Extract innerHTML from DOM (same as translation feature)
- **Rationale**: Gets actual rendered content including markdown-processed HTML
- **Implementation**: `document.querySelector('.chapter-content').innerHTML`
- **Alternatives Rejected**:
  - React node toString() (returns [object Object])
  - Raw markdown (not available in Docusaurus runtime)

### Decision 5: Technical Term Explanation Implementation
- **Decision**: AI adds inline parenthetical explanations when enabled
- **Rationale**: Non-intrusive, preserves reading flow, no additional UI needed
- **Example**: "ROS2 middleware" → "ROS2 (Robot Operating System 2) middleware"
- **Alternatives Rejected**:
  - Tooltips (requires complex UI hover state)
  - Footnotes (disrupts reading flow)
  - Sidebar glossary (clutters UI)

## Phase 1: Design Artifacts

### Key Entities (Updated for AI Personalization)

| Entity | Purpose | Storage |
|--------|---------|---------|
| UserProfile | software_experience, hardware_experience (existing) | Postgres |
| PersonalizationPreferences | reading_level, tech_explanations, example_density | React state (session) |
| PersonalizationRequest | chapter_id, content, preferences | Transient (API request) |
| PersonalizedContent | modified_content, original_content | React state (session) |

### API Contracts

#### NEW: POST /api/personalization/ai-personalize
**Purpose**: Generate AI-personalized content for a chapter

**Request**:
```json
{
  "chapter_id": "string",
  "content": "string (HTML)",
  "preferences": {
    "reading_level": "beginner | intermediate | advanced",
    "technical_explanations": true | false,
    "example_density": "minimal | normal | detailed"
  },
  "user_profile": {
    "software_experience": "beginner | intermediate | advanced",
    "hardware_experience": "none | basic | advanced"
  }
}
```

**Response (Success)**:
```json
{
  "success": true,
  "personalized_content": "string (HTML)",
  "chapter_id": "string",
  "preferences_applied": {
    "reading_level": "beginner",
    "technical_explanations": true,
    "example_density": "detailed"
  },
  "processing_time_ms": 5432
}
```

**Response (Error)**:
```json
{
  "success": false,
  "error": "string",
  "chapter_id": "string"
}
```

#### Existing Endpoints (Keep for Compatibility)
- `POST /api/personalization/activate` - Keep for bonus points (optional backward compat)
- `GET /api/personalization/status` - Keep for status tracking
- `GET /api/user/bonus-points` - Keep for gamification display

### Frontend State Design

```typescript
// Updated PersonalizationContext state
interface PersonalizationState {
  // Existing (keep for backward compatibility)
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  personalizationStatus: Record<string, PersonalizationStatus>;
  bonusPoints: UserBonusPoints;

  // NEW: AI Personalization
  preferences: {
    readingLevel: 'beginner' | 'intermediate' | 'advanced';
    technicalExplanations: boolean;
    exampleDensity: 'minimal' | 'normal' | 'detailed';
  };

  // Per-chapter personalized content (session only)
  personalizedContent: Record<string, {
    content: string;           // Personalized HTML
    originalContent: string;   // Original HTML for revert
    isPersonalized: boolean;
    personalizedAt: string;    // ISO timestamp
  }>;

  // Loading/error states
  isPersonalizing: boolean;
  personalizationError: string | null;
  isPanelOpen: boolean;
  activeChapterId: string | null;
}

// NEW Methods
personalizeChapter(chapterId: string, content: string): Promise<void>;
revertToOriginal(chapterId: string): void;
updatePreferences(prefs: Partial<Preferences>): void;
openPanel(chapterId: string): void;
closePanel(): void;
```

### UI Components Design

#### PersonalizationPanel (New Drawer Component)

```
┌─────────────────────────────────────┐
│ Personalize Content             [X] │
├─────────────────────────────────────┤
│                                     │
│ Reading Level:                      │
│ ┌─────────┬─────────┬─────────┐    │
│ │Beginner │Intermed.│Advanced │    │
│ └─────────┴─────────┴─────────┘    │
│ (selected: ●)                       │
│                                     │
├─────────────────────────────────────┤
│ Technical Terms:                    │
│ [✓] Show inline explanations        │
│                                     │
├─────────────────────────────────────┤
│ Example Density:                    │
│ ○ Minimal  ● Normal  ○ Detailed    │
│                                     │
├─────────────────────────────────────┤
│                                     │
│ [    Apply Personalization    ]     │
│                                     │
└─────────────────────────────────────┘
```

#### Button States Flow

1. **Default** (not personalized):
   - Text: "Personalize Content"
   - Color: Blue (#2196F3)
   - Action: Opens PersonalizationPanel

2. **Panel Open**:
   - Panel drawer slides in from right
   - User adjusts settings
   - Clicks "Apply Personalization"

3. **Processing**:
   - Text: "Personalizing..."
   - Shows loading spinner
   - Button disabled

4. **Personalized**:
   - Text: "Revert to Original"
   - Color: Green (#4CAF50) or toggle style
   - Action: Instantly reverts content

5. **Error**:
   - Shows error message
   - Retry button available

## AI Prompt Design

### System Prompt for Personalization Agent

```
You are an AI assistant that personalizes educational content about robotics and AI.

Your task is to adapt content based on the user's reading level while STRICTLY preserving:
- All headings and subheadings (exact text, exact hierarchy)
- All code blocks (unchanged)
- All images and diagrams
- The overall document structure
- All factual information (no fabrication)

Reading Level Guidelines:
- BEGINNER: Simplify language, add brief explanations for technical terms, include more examples
- INTERMEDIATE: Keep balanced explanations, moderate technical language
- ADVANCED: Use professional/concise language, minimal explanations, fewer examples

Technical Explanations:
- If ENABLED: Add brief inline parenthetical explanations for technical terms
  Example: "ROS2 middleware" → "ROS2 (Robot Operating System 2) middleware"
- If DISABLED: Keep original terms without modification

Example Density:
- MINIMAL: Show only essential examples needed for understanding
- NORMAL: Keep original examples unchanged
- DETAILED: Add supplementary examples where helpful

CRITICAL RULES:
1. NEVER change headings
2. NEVER modify code blocks
3. NEVER add information not present in original
4. NEVER remove sections entirely
5. Output must be valid HTML matching input structure
```

## Files to Modify

### Backend (4 files)

| File | Action | Key Changes |
|------|--------|-------------|
| `backend/src/services/personalization_agent.py` | CREATE | AI agent using OpenRouter, similar to translation_agent.py |
| `backend/src/personalization_config.py` | CREATE | Config for AI prompts and settings |
| `backend/endpoints/personalization.py` | MODIFY | Add `/ai-personalize` endpoint |
| `backend/main.py` | VERIFY | Ensure personalization router includes new endpoint |

### Frontend (7 files)

| File | Action | Key Changes |
|------|--------|-------------|
| `frontend/src/contexts/PersonalizationContext.tsx` | MODIFY | Add AI state, new methods |
| `frontend/src/services/personalizationService.ts` | MODIFY | Add aiPersonalize() function |
| `frontend/src/components/Personalization/PersonalizationButton.tsx` | MODIFY | Connect to panel, handle AI workflow |
| `frontend/src/components/Personalization/PersonalizationPanel.tsx` | CREATE | Settings drawer component |
| `frontend/src/components/Personalization/PersonalizationPanel.module.css` | CREATE | Panel styles |
| `frontend/src/components/Chapter/ChapterContent.tsx` | MODIFY | Display AI personalized content |
| `frontend/src/theme/DocItem/Layout/index.tsx` | MODIFY | Integrate PersonalizationPanel |

## Implementation Phases

### Phase 1: Backend AI Agent (P1 - Core)
1. Create `personalization_agent.py` copying pattern from `translation_agent.py`
2. Create `personalization_config.py` with prompts
3. Add `/api/personalization/ai-personalize` endpoint
4. Test with sample chapter content via curl

### Phase 2: Frontend State & Service (P1 - Core)
1. Update PersonalizationContext with AI state
2. Add `personalizeChapter()` and `revertToOriginal()` methods
3. Update personalizationService with aiPersonalize() call

### Phase 3: UI Components (P2 - UX)
1. Create PersonalizationPanel drawer component
2. Update PersonalizationButton for AI workflow
3. Add loading states and error handling
4. Style with CSS modules

### Phase 4: Content Rendering (P2 - Integration)
1. Update ChapterContent to conditionally display personalized content
2. Implement revert functionality (instant, from state)
3. Add visual indicator for personalized state

### Phase 5: Testing & Edge Cases (P3 - Polish)
1. Test all three reading levels
2. Test technical explanations toggle
3. Test example density modes
4. Test long chapters (10,000+ words)
5. Test error handling (AI unavailable)
6. Test unauthenticated users (button hidden)

## Complexity Tracking

No constitution violations identified. Implementation extends existing patterns:

| Pattern | Source | Reuse |
|---------|--------|-------|
| OpenRouter API integration | translation_agent.py | Copy pattern for personalization_agent.py |
| Context state management | TranslationContext.tsx | Copy pattern for AI state |
| Content extraction (innerHTML) | ChapterContent.tsx | Reuse for personalization |
| Loading/error handling | Translation button | Copy pattern |
| CSS modules styling | ChapterContent.module.css | Follow existing pattern |

## Next Steps

Run `/sp.tasks` to generate detailed implementation tasks from this plan.
