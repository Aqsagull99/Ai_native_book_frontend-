---
id: 1
title: professional-floating-rag-chatbot-ui
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
  - chatbot
  - rag
  - floating
  - documentation
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - frontend/src/components/FloatingRagChatbot.js
  - frontend/src/css/custom.css
  - frontend/src/ProvidersWrapper.tsx
tests_yaml: |
  - Component renders properly with chat bubbles
  - Typing animation displays correctly
  - Source citations expand/collapse
  - Responsive design works on mobile
  - Floating button toggles chat window
prompt_text: |
  You are a senior UI/UX designer and expert React frontend engineer.

  Task:
  Design a PROFESSIONAL floating RAG chatbot UI for a documentation-based learning platform.

  Core UX Direction:
  - Combine Option A (Floating Assistant) + Option C (Chat-style message bubbles)
  - The chatbot should feel modern, calm, and documentation-friendly
  - Similar quality level to ChatGPT, GitBook, and Notion assistants

  Required Features:
  1. Floating chat button fixed at bottom-right
     - Circular button with a professional React Icon
     - Click opens a slide-up chat window
     - Click again closes the window

  2. Chat Interface
     - User and bot messages displayed as chat bubbles
     - User bubbles aligned right
     - Bot bubbles aligned left
     - Clear visual distinction using color and spacing

  3. Typing Animation
     - While the bot is generating a response, show a typing indicator
     - Three animated dots inside a bot message bubble
     - Smooth and subtle animation (not flashy)

  4. Message Flow
     - Input field fixed at the bottom of the chat window
     - Messages scroll vertically
     - Auto-scroll to the latest message

  5. Sources / Citations
     - Bot messages may include sources
     - Display sources in a collapsible section inside the bot bubble
     - Use a professional book/reference icon

  6. Visual Style
     - Minimal, clean, professional
     - Neutral color palette suitable for technical documentation
     - Rounded corners, soft shadows
     - Comfortable spacing and readable typography

  7. Icons
     - Use professional React Icons (react-icons)
     - No emojis in the UI itself
     - Icons should enhance clarity, not distract

  8. CSS Requirements
     - Use custom CSS (NOT Tailwind, NOT Material UI)
     - Modular, readable, production-quality CSS
     - Responsive for desktop and mobile

  9. UX States
     - Empty state message (e.g. "Ask anything about this chapter")
     - Loading / thinking state with typing animation
     - Error state with friendly message

  Constraints:
  - Do NOT change backend logic
  - Focus only on UI/UX and frontend structure
  - Provide:
    • UX explanation
    • Component structure
    • React JSX example
    • Custom CSS example

  Goal:
  The final UI should transform the chatbot into a premium, professional assistant that enhances learning and feels trustworthy and polished.
response_text: |
  Successfully implemented a professional floating RAG chatbot UI with all requested features:
  - Created FloatingRagChatbot.js component with chat bubbles, typing animation, and source citations
  - Added professional CSS styling with glassmorphism effects
  - Implemented responsive design for desktop and mobile
  - Integrated with existing RAG API service
  - Updated ProvidersWrapper to use new component
  - Added react-icons dependency
outcome: |
  The floating RAG chatbot UI has been successfully implemented with:
  - Professional chat bubble interface with distinct user/bot styling
  - Smooth typing animation with three animated dots
  - Collapsible source citations with book icons
  - Responsive design that works on all screen sizes
  - Glassmorphism effects with backdrop filters
  - Floating toggle button with hover effects
  - Auto-scrolling to latest messages
  - Proper error handling and loading states
evaluation: |
  The implementation meets all requirements:
  ✓ Floating chat button at bottom-right
  ✓ Chat bubbles with proper alignment
  ✓ Typing animation with subtle dots
  ✓ Source citations with expand/collapse
  ✓ Professional styling with custom CSS
  ✓ Responsive design for mobile/desktop
  ✓ Integration with existing RAG API
---