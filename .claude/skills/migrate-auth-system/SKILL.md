---
name: migrate-auth-system
description: Migrate from other auth systems to Better Auth with schema conversion and user data migration
tools: Read, Edit, Bash, Grep
---

# Auth System Migration

## Instructions
1. Analyze the current authentication system (NextAuth, Auth.js, custom auth, etc.)
2. Plan the migration strategy considering user data, sessions, and existing providers
3. Create a schema mapping between the old system and Better Auth
4. Prepare database migration scripts
5. Migrate user data while preserving security
6. Update application code to use Better Auth
7. Test the migration thoroughly

## Migration Steps
1. Document the current auth system setup
2. Create a backup of the existing authentication data
3. Set up Better Auth alongside the current system
4. Map user data fields from old system to Better Auth schema
5. Create migration scripts for users, sessions, and accounts
6. Handle password hash compatibility if needed
7. Update API routes and middleware
8. Test authentication flows end-to-end
9. Switch over gradually with feature flags if possible

## Common Migration Scenarios
### From NextAuth.js:
- Migrate User, Account, Session, and VerificationToken tables
- Convert password hashes if needed (NextAuth typically uses bcrypt)
- Migrate OAuth account connections
- Update next-auth client calls to Better Auth client calls

### From Custom Auth:
- Map custom user schema to Better Auth schema
- Handle custom password hashing algorithms
- Migrate session management system
- Update authentication middleware/guards

## Data Migration Considerations
- Password hashes may need conversion or re-hashing
- OAuth provider tokens need to be preserved
- Session data needs to be compatible
- User roles/permissions need to be mapped
- Account linking for social providers