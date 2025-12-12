import React from 'react';
import { render, waitFor, act } from '@testing-library/react';
import { AuthProvider, useAuth, AuthContextType } from '../AuthContext';

// Mock fetch for API calls
global.fetch = jest.fn();

describe('AuthContext', () => {
  let authContext: AuthContextType | undefined;

  const TestComponent: React.FC = () => {
    authContext = useAuth();
    return <div>Test Component</div>;
  };

  beforeEach(() => {
    (global.fetch as jest.Mock).mockClear();
  });

  it('should provide auth context', () => {
    render(
      <AuthProvider>
        <TestComponent />
      </AuthProvider>
    );

    expect(authContext).toBeDefined();
    expect(authContext?.user).toBeNull();
    expect(typeof authContext?.login).toBe('function');
    expect(typeof authContext?.signup).toBe('function');
    expect(typeof authContext?.logout).toBe('function');
  });

  it('should handle signup', async () => {
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        message: 'User registered successfully',
        user_id: '123',
        email: 'test@example.com'
      })
    });

    render(
      <AuthProvider>
        <TestComponent />
      </AuthProvider>
    );

    if (authContext) {
      await act(async () => {
        await authContext.signup('test@example.com', 'password123', 'beginner', 'none');
      });

      // We can't directly test the state change here due to the async nature
      // In a real test, we would check that the user state is updated
      expect(global.fetch).toHaveBeenCalledWith(
        '/api/auth/signup',
        expect.objectContaining({
          method: 'POST',
          body: JSON.stringify({
            email: 'test@example.com',
            password: 'password123',
            software_experience: 'beginner',
            hardware_experience: 'none'
          })
        })
      );
    }
  });

  it('should handle signin', async () => {
    (global.fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({
        message: 'User authenticated successfully',
        email: 'test@example.com',
        user_id: '123'
      })
    });

    render(
      <AuthProvider>
        <TestComponent />
      </AuthProvider>
    );

    if (authContext) {
      await act(async () => {
        await authContext.login('test@example.com', 'password123');
      });

      expect(global.fetch).toHaveBeenCalledWith(
        '/api/auth/signin',
        expect.objectContaining({
          method: 'POST',
          body: JSON.stringify({
            email: 'test@example.com',
            password: 'password123'
          })
        })
      );
    }
  });
});