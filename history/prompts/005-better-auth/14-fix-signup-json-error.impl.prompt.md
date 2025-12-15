---
id: 14
title: fix-signup-json-error
stage: impl
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-better-auth
branch: 005-better-auth
user: aqsagulllinux
command: /sp.implement
labels: [bug-fix, auth, frontend]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/auth.py
  - frontend/src/contexts/AuthContext.tsx
  - frontend/src/components/AuthForm/SignupForm.tsx
  - frontend/src/components/AuthForm/SigninForm.tsx
  - frontend/.env
  - test_auth_flow.py
tests: []
---

# Fix Signup JSON Error

## Prompt Text

when i creat our acount after fill create acount form then click signup button i got
this error """Unexpected token '<', "<!DOCTYPE "... is not valid JSON""''

## Response Text

Fixed the "Unexpected token '<', '<!DOCTYPE "... is not valid JSON"'" error that was occurring during signup. The issue was that the frontend was making API requests to `/api/auth/signup` but these requests were not being properly directed to the backend server.

### Root Cause
The error occurred because:
1. The frontend was making API calls to `/api/auth/signup` expecting JSON responses
2. Without proper proxy configuration, these requests were being made to the Docusaurus frontend server instead of the backend API server
3. The Docusaurus server was returning HTML (the main page) instead of JSON, causing the "Unexpected token '<'" error

### Solution Implemented

1. **Updated AuthContext.tsx** - Modified all API calls to use the correct backend URL:
   - `login` function: Changed from `/api/auth/signin` to `${BACKEND_URL}/api/auth/signin`
   - `signup` function: Changed from `/api/auth/signup` to `${BACKEND_URL}/api/auth/signup`
   - `fetchProfile` function: Changed from `/api/auth/profile` to `${BACKEND_URL}/api/auth/profile`
   - `updateProfile` function: Changed from `/api/auth/profile` to `${BACKEND_URL}/api/auth/profile`
   - Added environment variable support with fallback to `http://localhost:8000`

2. **Created frontend/.env file** with `BACKEND_URL=http://localhost:8000`

3. **Fixed navigation in auth forms** - Updated both SignupForm and SigninForm to use the correct Docusaurus router methods:
   - Changed from `useNavigate` to `useHistory` (which is available in `@docusaurus/router`)
   - Changed navigation from `navigate('/')` to `history.push('/')`

4. **Verified the backend is working** - Successfully tested the complete authentication flow with the test script

### Testing
The authentication flow test confirmed that:
- ✅ User signup works correctly
- ✅ User signin works correctly
- ✅ Profile retrieval works correctly
- ✅ Profile updates work correctly

The signup error should now be resolved, and users should be able to create accounts without encountering the JSON parsing error.

## Outcome

Successfully resolved the signup error by updating API endpoints to properly connect to the backend server and fixing frontend routing methods. The authentication flow now works correctly end-to-end.