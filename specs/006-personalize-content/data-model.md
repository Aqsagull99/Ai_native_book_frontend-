# Data Model: AI-Powered Chapter Personalization

**Updated**: 2025-12-27
**Focus**: Session-only AI personalization (NO database persistence for personalized content)

---

## Overview

The AI personalization feature uses a **hybrid storage model**:
- **Existing DB entities** (UserProfile, UserPersonalizationPreference, UserBonusPoints) remain for backward compatibility
- **New session-only entities** (PersonalizationPreferences, PersonalizedContent) are stored in React state only

---

## Existing Database Entities (Keep for Compatibility)

### Entity: UserProfile (Existing)
**Location**: `backend/models.py`
**Purpose**: Stores user experience levels used as defaults for AI personalization

| Field | Type | Description |
|-------|------|-------------|
| `id` | Integer, PK | Unique identifier |
| `user_id` | String, FK, Unique | Links to auth user |
| `email` | String, Unique | User email |
| `software_experience` | String | beginner \| intermediate \| advanced |
| `hardware_experience` | String | none \| basic \| advanced |
| `created_at` | DateTime | Creation timestamp |
| `updated_at` | DateTime | Last update timestamp |

**Used For**: Default reading level mapping in AI personalization

### Entity: UserPersonalizationPreference (Existing - Optional)
**Location**: `backend/models.py`
**Purpose**: Tracks which chapters user has personalized (for analytics)

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID, PK | Unique identifier |
| `user_id` | String, FK | Links to auth user |
| `chapter_id` | String | Chapter identifier |
| `preferences` | JSON Text | Preferences used (optional log) |
| `created_at` | DateTime | When personalized |
| `updated_at` | DateTime | Last update |

**Note**: This entity is optional for AI personalization. Can be used to track personalization history but personalized content is NOT stored here.

### Entity: UserBonusPoints (Existing - Optional)
**Location**: `backend/models.py`
**Purpose**: Tracks gamification points (optional, kept for backward compatibility)

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID, PK | Unique identifier |
| `user_id` | String, FK | Links to auth user |
| `chapter_id` | String | Chapter identifier |
| `points_earned` | Integer | Points awarded (default: 50) |
| `earned_at` | DateTime | When earned |
| `is_valid` | Boolean | Validation flag |

**Note**: Optional for AI personalization. Can be used alongside AI feature for gamification.

---

## NEW: Session-Only Entities (React State)

### Entity: PersonalizationPreferences
**Location**: `frontend/src/contexts/PersonalizationContext.tsx`
**Storage**: React state (session memory only)
**Purpose**: User's current personalization settings

```typescript
interface PersonalizationPreferences {
  readingLevel: 'beginner' | 'intermediate' | 'advanced';
  technicalExplanations: boolean;
  exampleDensity: 'minimal' | 'normal' | 'detailed';
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `readingLevel` | enum | From UserProfile | Controls language complexity |
| `technicalExplanations` | boolean | true for beginner | Show inline term explanations |
| `exampleDensity` | enum | 'normal' | How many examples to include |

**Default Mapping from UserProfile**:
```typescript
const mapProfileToDefaults = (profile: UserProfile): PersonalizationPreferences => ({
  readingLevel: profile.software_experience || 'intermediate',
  technicalExplanations: profile.software_experience === 'beginner',
  exampleDensity: 'normal'
});
```

### Entity: PersonalizedContent
**Location**: `frontend/src/contexts/PersonalizationContext.tsx`
**Storage**: React state (session memory only)
**Purpose**: Stores AI-personalized content per chapter

```typescript
interface PersonalizedContentEntry {
  content: string;           // AI-modified HTML content
  originalContent: string;   // Original HTML for instant revert
  isPersonalized: boolean;   // Flag indicating personalization active
  personalizedAt: string;    // ISO timestamp of personalization
  preferencesUsed: PersonalizationPreferences;  // Settings used
}

type PersonalizedContentMap = Record<string, PersonalizedContentEntry>;
// Key: chapter_id (e.g., "module-1/chapter-1-introduction")
```

| Field | Type | Description |
|-------|------|-------------|
| `content` | string | AI-personalized HTML from LLM |
| `originalContent` | string | Original HTML for revert |
| `isPersonalized` | boolean | True when personalized |
| `personalizedAt` | string | ISO timestamp |
| `preferencesUsed` | object | Settings applied |

**Lifecycle**:
1. Created when user clicks "Apply Personalization"
2. Updated if user re-personalizes with different settings
3. Deleted on revert or page refresh/logout

---

## API Request/Response Models

### PersonalizationRequest (Transient)
**Location**: API request body
**Purpose**: Request AI personalization from backend

```typescript
interface PersonalizationRequest {
  chapter_id: string;
  content: string;  // HTML content to personalize
  preferences: {
    reading_level: 'beginner' | 'intermediate' | 'advanced';
    technical_explanations: boolean;
    example_density: 'minimal' | 'normal' | 'detailed';
  };
  user_profile: {
    software_experience: string;
    hardware_experience: string;
  };
}
```

### PersonalizationResponse (Transient)
**Location**: API response body
**Purpose**: Return personalized content

```typescript
interface PersonalizationResponse {
  success: boolean;
  personalized_content: string;  // AI-modified HTML
  chapter_id: string;
  preferences_applied: {
    reading_level: string;
    technical_explanations: boolean;
    example_density: string;
  };
  processing_time_ms: number;
}
```

---

## State Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     USER SESSION START                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Load UserProfile from DB                                        │
│ - software_experience                                           │
│ - hardware_experience                                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Initialize PersonalizationPreferences (React State)             │
│ - Map profile to defaults                                       │
│ - readingLevel = software_experience                            │
│ - technicalExplanations = (level === 'beginner')               │
│ - exampleDensity = 'normal'                                     │
└─────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┴────────────────────┐
         ▼                                         ▼
┌─────────────────────┐               ┌─────────────────────────┐
│ User views chapter  │               │ User adjusts settings   │
│ (no personalization)│               │ in PersonalizationPanel │
└─────────────────────┘               └─────────────────────────┘
         │                                         │
         └────────────────────┬────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ User clicks "Apply Personalization"                             │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ POST /api/personalization/ai-personalize                        │
│ - Send chapter HTML + preferences                               │
│ - Backend calls OpenRouter LLM                                  │
│ - LLM returns personalized HTML                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Store in PersonalizedContent (React State)                      │
│ - personalizedContent[chapter_id] = { content, original, ... }  │
│ - Display personalized HTML                                     │
└─────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┴────────────────────┐
         ▼                                         ▼
┌─────────────────────┐               ┌─────────────────────────┐
│ User clicks         │               │ User navigates away     │
│ "Revert to Original"│               │ or refreshes page       │
└─────────────────────┘               └─────────────────────────┘
         │                                         │
         ▼                                         ▼
┌─────────────────────┐               ┌─────────────────────────┐
│ Instant revert:     │               │ Session state cleared   │
│ delete from state   │               │ (personalization lost)  │
└─────────────────────┘               └─────────────────────────┘
```

---

## Validation Rules

1. **Authentication Required**: Only authenticated users can trigger AI personalization
2. **Session-Only Storage**: Personalized content is NEVER saved to database
3. **Original Preserved**: Original content must be stored for instant revert
4. **Structure Preserved**: AI must preserve all headings, code blocks, images
5. **No Fabrication**: AI must not add information not in original content

---

## Migration Notes

No database migration required. The AI personalization feature:
- Uses existing UserProfile for defaults
- Stores all personalization data in React state
- Does not modify existing database tables
- Can optionally log to UserPersonalizationPreference for analytics
