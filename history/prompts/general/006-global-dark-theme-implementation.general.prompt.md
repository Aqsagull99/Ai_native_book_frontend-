---
id: 006
title: "Global Dark Theme Implementation"
stage: general
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "007-website-content-embedding"
branch: "007-website-content-embedding"
user: "user"
command: "I am working on a Docusaurus website.

I already styled my homepage hero section with:
- Animated gradient background (dark yellow, dark green, dark pink)
- Glassmorphism UI
- Dark theme design

Now I want to apply the SAME design language to the following pages:
- Profile page
- Sign Up page
- Sign In page

Global Theme Requirement:
- Change the GLOBAL background color of the site to BLACK.
- Remove white/light backgrounds completely.
- All pages should follow a dark theme.

Page-Specific Requirements:

1. Profile Page:
- Background should be black.
- Use glassmorphism styling for the profile container:
  - Semi-transparent dark background
  - Blur effect
  - Soft border
  - Subtle shadow
- Text should be clearly readable (light text on dark background).
- Layout should feel clean and premium.

2. Sign Up Page:
- Background should be black.
- The sign-up form should follow the SAME styling as the homepage hero:
  - Glassmorphism card
  - Animated gradient accents (dark yellow, dark green, dark pink)
- Form inputs should be well-styled:
  - Dark input backgrounds
  - Light text
  - Clear focus states
- Primary action button should be visually highlighted.

3. Sign In Page:
- Same styling as Sign Up page for consistency.
- Glassmorphism form container.
- Dark theme inputs and buttons.
- Smooth hover and focus animations.

Consistency Rules:
- All pages must visually match the homepage hero styling.
- Use the same color palette and animation style.
- No white backgrounds anywhere.
- Design should feel modern, futuristic, and AI/robotics themed.

Technical Constraints:
- Must work with Docusaurus pages.
- Use React + CSS or CSS Modules.
- Do not break existing routing or navbar.
- Keep styling reusable and clean.

Please provide:
- CSS (or CSS module) for:
  - Global black background
  - Glassmorphism cards
  - Form styling (inputs, labels, buttons)
- Example JSX structure for Profile, Sign In, and Sign Up pages
- Ensure responsive design (mobile + desktop)."
labels:
  - "ui-design"
  - "docusaurus"
  - "dark-theme"
  - "glassmorphism"
  - "authentication"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "frontend/src/css/custom.css"
  - "frontend/src/pages/profile.tsx"
  - "frontend/src/pages/signin.tsx"
  - "frontend/src/pages/signup.tsx"
  - "frontend/src/components/AuthForm/AuthForm.css"
tests_yaml: []
---

# Prompt History Record

## PROMPT_TEXT
```
I am working on a Docusaurus website.

I already styled my homepage hero section with:
- Animated gradient background (dark yellow, dark green, dark pink)
- Glassmorphism UI
- Dark theme design

Now I want to apply the SAME design language to the following pages:
- Profile page
- Sign Up page
- Sign In page

Global Theme Requirement:
- Change the GLOBAL background color of the site to BLACK.
- Remove white/light backgrounds completely.
- All pages should follow a dark theme.

Page-Specific Requirements:

1. Profile Page:
- Background should be black.
- Use glassmorphism styling for the profile container:
  - Semi-transparent dark background
  - Blur effect
  - Soft border
  - Subtle shadow
- Text should be clearly readable (light text on dark background).
- Layout should feel clean and premium.

2. Sign Up Page:
- Background should be black.
- The sign-up form should follow the SAME styling as the homepage hero:
  - Glassmorphism card
  - Animated gradient accents (dark yellow, dark green, dark pink)
- Form inputs should be well-styled:
  - Dark input backgrounds
  - Light text
  - Clear focus states
- Primary action button should be visually highlighted.

3. Sign In Page:
- Same styling as Sign Up page for consistency.
- Glassmorphism form container.
- Dark theme inputs and buttons.
- Smooth hover and focus animations.

Consistency Rules:
- All pages must visually match the homepage hero styling.
- Use the same color palette and animation style.
- No white backgrounds anywhere.
- Design should feel modern, futuristic, and AI/robotics themed.

Technical Constraints:
- Must work with Docusaurus pages.
- Use React + CSS or CSS Modules.
- Do not break existing routing or navbar.
- Keep styling reusable and clean.

Please provide:
- CSS (or CSS module) for:
  - Global black background
  - Glassmorphism cards
  - Form styling (inputs, labels, buttons)
- Example JSX structure for Profile, Sign In, and Sign Up pages
- Ensure responsive design (mobile + desktop).
```

## RESPONSE_TEXT

I have successfully implemented the global dark theme across all requested pages with consistent design language:

### Changes Made:

1. **Global Black Background Theme**:
   - Updated `frontend/src/css/custom.css` to set global black background
   - Configured dark theme variables for consistent styling
   - Added animated gradient background animation

2. **Reusable Glassmorphism Components**:
   - Created `.glassmorphism-card` CSS class with:
     - Semi-transparent background (rgba(255,255,255,0.1))
     - Backdrop blur effect (10px)
     - Soft border and subtle shadow
   - Applied consistent styling across all pages

3. **Profile Page Styling**:
   - Wrapped content in glassmorphism card container
   - Ensured readable light text on dark background
   - Maintained all existing functionality

4. **Sign In & Sign Up Page Styling**:
   - Updated AuthForm component with dark theme styling
   - Applied glassmorphism effect to form containers
   - Styled form inputs with dark backgrounds and light text
   - Enhanced button styling with hover animations and visual highlights

5. **Consistent Color Palette**:
   - Dark yellow (#f9a825), dark green (#2e7d32), dark pink (#ad1457) accents
   - Proper contrast ratios for readability
   - Consistent button and form styling across all pages

6. **Responsive Design**:
   - Added media queries for mobile and tablet layouts
   - Proper padding and spacing adjustments for smaller screens
   - Maintained glassmorphism effect on all screen sizes

### Files Modified:
- `frontend/src/css/custom.css`: Global theme, glassmorphism styles, and responsive design
- `frontend/src/components/AuthForm/AuthForm.css`: Dark theme for auth forms
- `frontend/src/pages/profile.tsx`: Added glassmorphism container
- `frontend/src/pages/signin.tsx`: Added glassmorphism container
- `frontend/src/pages/signup.tsx`: Added glassmorphism container

All pages now have a consistent dark theme with glassmorphism effects that match the homepage hero styling, with proper responsive design for mobile and desktop.