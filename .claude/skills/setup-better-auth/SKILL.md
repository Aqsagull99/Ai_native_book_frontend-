---
name: setup-better-auth
description: Initialize Better Auth in a project with proper configuration, database setup, and provider configuration
tools: Read, Edit, Bash
---

# Better Auth Setup

## Instructions
1. Check if Better Auth is already installed in the project
2. Install Better Auth if not present:
   ```bash
   npm install better-auth
   ```
3. Create the main auth configuration file (usually `src/auth.ts` or similar)
4. Set up the basic configuration with required options
5. Configure the database connection
6. Set up the client-side configuration if needed
7. Create necessary database tables using migrations

## Configuration Steps
1. Create auth server instance with basic options
2. Configure the database adapter (PostgreSQL, MySQL, SQLite, etc.)
3. Set up session options
4. Configure social providers if needed
5. Add the auth API routes to your application

## Example Setup
```typescript
import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapter-drizzle";
import { db } from "./db"; // your database instance

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    // your database options
  }),
  secret: process.env.AUTH_SECRET,
  emailAndPassword: {
    enabled: true,
  },
  socialProviders: {
    // configure social providers here
  },
});
```