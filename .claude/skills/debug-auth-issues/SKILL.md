---
name: debug-auth-issues
description: Troubleshoot authentication problems including session debugging and token validation
tools: Read, Grep, Bash
---

# Authentication Issue Debugging

## Instructions
1. Identify the specific authentication issue or error message
2. Check server logs for error details
3. Verify configuration settings
4. Test authentication flows step by step
5. Validate session and token states
6. Check network connectivity and redirects
7. Verify environment variables and credentials
8. Document the solution for future reference

## Common Issues and Solutions

### Session Issues
- Check if session cookie is being set properly
- Verify domain and path settings for cookies
- Ensure HTTPS is configured properly in production
- Check session expiration settings

### OAuth Provider Issues
- Verify client ID and secret are correct
- Check that redirect URLs match provider configuration
- Ensure provider is properly enabled in Better Auth config
- Test network connectivity to provider

### Database Issues
- Verify database connection is working
- Check that required tables exist
- Ensure proper database permissions
- Look for migration issues

### Configuration Issues
- Check that AUTH_SECRET is set and consistent
- Verify all required environment variables
- Ensure configuration matches environment (dev/prod)
- Check for typos in configuration

## Debugging Steps
1. Reproduce the issue in a controlled environment
2. Enable detailed logging if possible
3. Check the network tab in browser dev tools
4. Validate request/response headers and cookies
5. Test with a minimal configuration to isolate the issue
6. Gradually add complexity back to identify the problem

## Tools for Debugging
- Browser developer tools (Network tab for requests, Application tab for cookies/storage)
- Server logs
- Database queries to check user/session data
- Environment variable validation
- Configuration validation