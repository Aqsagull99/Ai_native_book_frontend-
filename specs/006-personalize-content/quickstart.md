# Quickstart: Personalized Chapter Content

## Overview
This guide will help you set up and run the personalization feature that allows users to customize chapter content and earn bonus points. The feature includes a prominent colored personalization button at the start of each chapter that triggers content customization based on user profile data with visual styling changes to distinguish personalized content.

## Prerequisites
- Python 3.12+
- Node.js 18+
- PostgreSQL or Neon Postgres database
- Better-Auth configured

## Backend Setup

1. **Install Python dependencies**:
   ```bash
   pip install fastapi sqlalchemy asyncpg python-multipart
   ```

2. **Update database models**:
   Add the UserPersonalizationPreference and UserBonusPoints models to your existing models.py file based on the data-model.md specifications.

3. **Create database tables**:
   Run the database migration to create the personalization tables:
   ```bash
   python create_personalization_tables.py
   ```

4. **Add API endpoints**:
   Create the personalization endpoints in `backend/endpoints/personalization.py` based on the API contract specifications in contracts/personalization-api.yaml.

5. **Register endpoints**:
   Include the personalization endpoints in your main FastAPI application.

## Frontend Setup

1. **Install TypeScript dependencies**:
   ```bash
   npm install @types/react
   ```

2. **Add personalization components**:
   - Create `PersonalizationButton.tsx` component that triggers the personalization API with distinctive styling (orange or teal color as specified)
   - Create `ChapterContent.tsx` component that displays personalized content with visual styling changes (like color highlights)
   - Update `ChapterPage.tsx` to include the personalization button at the start of each chapter

3. **Add service layer**:
   Create `personalizationService.ts` with functions to call the personalization API endpoints.

4. **Add context**:
   Create `PersonalizationContext.tsx` to manage personalization state across components.

## Environment Configuration

Add the following environment variables:
```env
PERSONALIZATION_ENABLED=true
BONUS_POINTS_PER_CHAPTER=50
```

## Running the Application

1. **Start the backend**:
   ```bash
   uvicorn main:app --reload
   ```

2. **Start the frontend**:
   ```bash
   npm start
   ```

## Testing

1. **Unit tests**:
   ```bash
   pytest tests/personalization/
   ```

2. **Integration tests**:
   Verify that personalization works end-to-end by:
   - Logging in as a user
   - Navigating to a chapter
   - Clicking the prominent colored personalization button at the start
   - Confirming content is personalized with visual styling changes
   - Verifying 50 bonus points are awarded