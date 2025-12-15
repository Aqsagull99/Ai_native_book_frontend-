# Data Model: Personalized Chapter Content

## Entity: UserPersonalizationPreference
**Description**: Stores user profile customizations that are applied to specific chapter content

### Fields:
- `id` (UUID, Primary Key): Unique identifier for the personalization preference
- `user_id` (UUID, Foreign Key): Reference to the user who owns this preference (links to auth system)
- `chapter_id` (String): Identifier for the chapter being personalized (based on existing docs folder structure)
- `preferences` (JSON): User-specific content customization settings (profile customizations applied to chapter)
- `created_at` (DateTime): Timestamp when personalization was first activated
- `updated_at` (DateTime): Timestamp when preferences were last modified

### Relationships:
- One-to-One with User (via user_id)
- Validation: Unique constraint on (user_id, chapter_id) to prevent duplicate personalization

## Entity: UserBonusPoints
**Description**: Tracks earned points from personalization activities with timestamp and chapter reference

### Fields:
- `id` (UUID, Primary Key): Unique identifier for the bonus point record
- `user_id` (UUID, Foreign Key): Reference to the user who earned these points (links to auth system)
- `chapter_id` (String): Identifier for the chapter where points were earned
- `points_earned` (Integer): Number of points earned (50 per chapter as specified)
- `earned_at` (DateTime): Timestamp when points were awarded
- `is_valid` (Boolean): Whether these points are still valid (to handle potential abuse)

### Relationships:
- One-to-One with User (via user_id)
- Validation: Unique constraint on (user_id, chapter_id) to prevent duplicate points

## Entity: Chapter
**Description**: Represents a content section from the existing docs folder structure that can be personalized with associated personalization status

### Fields:
- `id` (String, Primary Key): Unique identifier for the chapter (based on existing docs folder structure)
- `title` (String): Chapter title
- `content_path` (String): File path to the chapter content in the docs folder
- `default_content` (Text): Default chapter content before personalization
- `personalization_template` (JSON): Template for how content can be personalized using user profile customizations

### Relationships:
- One-to-Many with UserPersonalizationPreference (via chapter_id)
- One-to-Many with UserBonusPoints (via chapter_id)

## Validation Rules:
1. A user can only earn points once per chapter (enforced by unique constraint on UserBonusPoints)
2. Points earned must be exactly 50 per chapter (as specified in requirements)
3. User must be authenticated to create personalization preferences
4. Chapter must exist before personalization can be applied
5. Personalization preferences must be based on user profile customizations (as clarified)

## State Transitions:
- Chapter: [Default Content] → [Personalized Content] when personalization is activated
- UserBonusPoints: [Not Earned] → [Earned] when personalization is first activated
- UserPersonalizationPreference: [Not Set] → [Customized] when personalization is activated