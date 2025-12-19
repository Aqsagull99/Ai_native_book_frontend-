---
id: 3
title: modern-homepage-hero-section
stage: general
date_iso: '2025-12-19'
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: claude
command: sp.implement
labels:
  - ui
  - homepage
  - hero
  - docusaurus
  - robotics
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - frontend/docusaurus.config.ts
  - frontend/src/pages/index.tsx
  - frontend/src/pages/index.module.css
tests_yaml: |
  - Hero section renders with correct title
  - Subtitle content is technically grounded
  - Primary/secondary buttons function
  - Technical SVG graphic displays
  - Responsive design works on mobile
prompt_text: |
  You are a senior UI/UX designer and React frontend expert.

  Task:
  Design a modern, professional HOMEPAGE HERO SECTION for a technical book website titled:

  "Physical AI & Humanoid Robotics"

  Context:
  - The website hosts a technical book focused on:
    • Physical AI
    • Humanoid robotics
    • ROS / autonomy
    • Real-world robotics systems
  - The homepage should immediately communicate credibility, depth, and innovation
  - Audience includes students, engineers, researchers, and developers

  Hero Section Requirements:

  1. Main Title
     - Prominently display:
       "Physical AI & Humanoid Robotics"
     - Bold, confident typography
     - Feels academic + industry-grade (not marketing fluff)

  2. Subtitle / Description
     - Write a concise but powerful subtitle explaining the book
     - Should emphasize:
       • Learning by building
       • Real-world robotics systems
       • AI + hardware + software integration
     - Clear, inspiring, and technically grounded

  3. Primary CTA Button
     - Button text:
       "Start Reading"
       OR
       "Read the Book"
     - Visually strong and immediately noticeable
     - Designed to guide users into the documentation/book chapters

  4. Secondary CTA (optional)
     - Example:
       "Explore Chapters"
       "View Architecture"
     - Subtle but accessible

  5. Visual Direction
     - No generic stock images
     - Prefer:
       • Abstract robotics visuals
       • Minimal line graphics
       • Subtle grid or technical patterns
     - Clean, calm, futuristic but professional

  6. Layout
     - Two-column layout on desktop:
       • Left: text + buttons
       • Right: visual / abstract illustration
     - Stacked layout on mobile
     - Balanced whitespace

  7. Styling
     - Documentation-friendly color palette
     - Soft shadows, rounded corners
     - Professional typography hierarchy
     - Looks good in both light and dark themes

  8. Technology Constraints
     - React + JSX
     - Compatible with Docusaurus homepage
     - Use custom CSS (NOT Tailwind, NOT Material UI)
     - Buttons should be reusable React components

  Deliverables:
  - UX explanation
  - Suggested layout structure
  - Rewritten hero title + subtitle
  - Button copy
  - JSX example for the hero section
  - Custom CSS example

  Goal:
  Create a homepage that instantly feels like a serious, high-quality technical robotics book — not a generic template — and motivates users to start reading immediately.
response_text: |
  Successfully implemented a modern, professional homepage hero section with:
  - Updated site tagline to be technically grounded
  - Enhanced hero section with abstract technical SVG graphic
  - Added dual CTA buttons ("Start Reading" primary, "Explore Chapters" secondary)
  - Professional styling with glassmorphism effects
  - Responsive design for all screen sizes
  - Technical SVG pattern representing robotics systems
  - Improved typography hierarchy and spacing
outcome: |
  The homepage hero section has been successfully redesigned with:
  - Professional technical SVG graphic replacing generic image
  - Dual CTA buttons with clear hierarchy
  - Technically-grounded subtitle content
  - Modern glassmorphism design with hover effects
  - Responsive layout that works on all devices
  - Consistent styling with the overall site theme
evaluation: |
  The implementation meets all requirements:
  ✓ Professional technical visual element
  ✓ Technically grounded content
  ✓ Dual CTA buttons with clear hierarchy
  ✓ Responsive two-column layout
  ✓ Modern, professional styling
  ✓ Compatible with Docusaurus
---