# Research Summary: AI-Powered Chapter Personalization

**Updated**: 2025-12-27
**Focus**: Transformed from gamification/bonus-points to AI-powered content personalization

---

## Decision 1: AI Model for Personalization

**Decision**: Use OpenRouter with existing mistralai/devstral-2512:free model

**Rationale**:
- Already integrated and tested in translation feature (`translation_agent.py`)
- Free tier available for hackathon demo
- Consistent architecture across AI features
- Proven to handle HTML content transformation

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|-----------------|
| Google Gemini (existing RAG) | Different integration pattern, would add complexity |
| Claude API | Cost concerns for demo/hackathon |
| OpenAI GPT | Requires new API key setup |
| Local LLM | Performance/deployment complexity |

---

## Decision 2: Personalization Storage Strategy

**Decision**: Session-only storage in frontend React state (NO database persistence)

**Rationale**:
- Per spec requirement: "No permanent saving of personalized content"
- Personalized content is temporary and regenerated on each request
- Reduces database load and storage costs
- Simpler implementation without migration concerns

**Implementation**:
```typescript
// PersonalizationContext state
personalizedContent: Record<string, {
  content: string;           // AI-modified HTML
  originalContent: string;   // For instant revert
  isPersonalized: boolean;
  personalizedAt: string;
}>;
```

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|-----------------|
| Database storage | Violates spec constraint |
| LocalStorage | Risk of stale content, quota limits |
| IndexedDB | Over-engineering for session-only need |

---

## Decision 3: User Preferences Source

**Decision**: Combined approach - Profile defaults + click-time customization panel

**Rationale**:
- Best UX balance between speed and flexibility
- User sees personalized defaults from their profile
- Can adjust per-chapter if needed
- Reduces repetitive configuration

**Implementation Flow**:
1. User clicks "Personalize Content" button
2. PersonalizationPanel drawer opens
3. Defaults loaded from `user.software_experience` and `user.hardware_experience`
4. User can adjust reading level, tech explanations, example density
5. Click "Apply" triggers AI personalization

**Default Mapping**:
| User Profile | Default Reading Level |
|--------------|----------------------|
| software_experience = beginner | Beginner |
| software_experience = intermediate | Intermediate |
| software_experience = advanced | Advanced |

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|-----------------|
| Profile-only (no UI) | No flexibility for per-chapter adjustment |
| Always ask (no defaults) | Slow, repetitive UX |
| Global settings page | Extra navigation, less contextual |

---

## Decision 4: Content Extraction Method

**Decision**: Extract innerHTML from DOM element (same as translation feature)

**Rationale**:
- Proven pattern from working translation feature
- Gets fully rendered HTML including processed markdown
- Preserves all structure, images, code blocks
- Consistent approach across features

**Implementation**:
```typescript
const contentElement = document.querySelector('.chapter-content');
const htmlContent = contentElement?.innerHTML || '';
```

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|-----------------|
| React node.toString() | Returns `[object Object]` - doesn't work |
| Raw markdown source | Not available at runtime in Docusaurus |
| Server-side extraction | Adds complexity, slower |

---

## Decision 5: Technical Term Explanations

**Decision**: AI adds inline parenthetical explanations when toggle enabled

**Rationale**:
- Non-intrusive, preserves reading flow
- No additional UI components needed
- AI can contextually identify technical terms
- Easy to toggle on/off

**Example Transformation**:
```
Original: "ROS2 middleware handles inter-process communication"
Beginner with explanations: "ROS2 (Robot Operating System 2, a popular robotics framework) middleware handles inter-process communication (the way different programs talk to each other)"
```

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|-----------------|
| Hover tooltips | Requires complex UI state, accessibility concerns |
| Footnotes | Disrupts reading flow, awkward for digital |
| Sidebar glossary | Clutters UI, requires scrolling |
| Underline with modal | Heavy UI, interrupts reading |

---

## Decision 6: Example Density Implementation

**Decision**: AI adjusts example inclusion based on density setting

**Rationale**:
- Flexible control for different learning styles
- AI can identify examples vs core content
- Preserves document structure

**Density Modes**:
| Mode | AI Behavior |
|------|-------------|
| Minimal | Remove supplementary examples, keep only essential demonstrations |
| Normal | Keep original examples unchanged |
| Detailed | Add clarifying examples, expand on complex concepts |

---

## Decision 7: Revert Functionality

**Decision**: Instant revert from frontend state (no API call)

**Rationale**:
- Per spec: "Users can revert to original content instantly (under 1 second)"
- Original content stored in React state
- No network latency
- Simple implementation

**Implementation**:
```typescript
revertToOriginal(chapterId: string): void {
  setPersonalizedContent(prev => {
    const updated = { ...prev };
    delete updated[chapterId];
    return updated;
  });
}
```

---

## Technical Patterns to Reuse

Based on codebase exploration, these patterns will be copied:

| Pattern | Source File | Reuse For |
|---------|-------------|-----------|
| OpenRouter API call | `backend/src/services/translation_agent.py` | AI personalization agent |
| Config validation | `backend/src/translation_config.py` | Personalization config |
| Context state + reducer | `frontend/src/contexts/TranslationContext.tsx` | AI state management |
| Content extraction | `frontend/src/components/Chapter/ChapterContent.tsx` | Get chapter HTML |
| CSS modules | `frontend/src/components/Chapter/ChapterContent.module.css` | Panel styles |
| Auth header | `frontend/src/services/personalizationService.ts` | API calls |

---

## Technical Unknowns Resolved

| Unknown | Resolution |
|---------|------------|
| How to call LLM for personalization | Reuse OpenRouter pattern from translation_agent.py |
| How to store personalized content | React state only (session) |
| How to get user preferences | Combined: profile defaults + customization panel |
| How to extract chapter content | innerHTML from DOM (proven in translation) |
| How to add term explanations | AI inline parenthetical additions |
| How to implement instant revert | Store original in state, swap on revert |
| How to handle long chapters | Same chunking strategy as translation if needed |

---

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| AI hallucination | System prompt strictly forbids adding new info |
| Slow personalization | Loading indicator, timeout at 30s |
| Structure corruption | AI prompt requires preserving headings/code |
| API unavailable | Graceful error with retry option |
| Large content | Frontend chunking if >10k words |
