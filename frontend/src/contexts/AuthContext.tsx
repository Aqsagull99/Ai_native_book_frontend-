import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  software_experience: string;
  hardware_experience: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => void;
  signup: (email: string, password: string, software_experience: string, hardware_experience: string) => Promise<void>;
  updateProfile: (software_experience?: string, hardware_experience?: string) => Promise<void>;
  fetchProfile: (email: string) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Check for existing session on app load
  useEffect(() => {
    const token = localStorage.getItem('token');
    if (token) {
      // In a real implementation, we would validate the token with the backend
      // For now, we'll just fetch the user profile if we have an email in storage
      const email = localStorage.getItem('userEmail');
      if (email) {
        fetchProfile(email).catch(() => {
          // If fetching profile fails, clear the stored token
          localStorage.removeItem('token');
          localStorage.removeItem('userEmail');
        });
      }
    }
    setLoading(false);
  }, []);

  const login = async (email: string, password: string) => {
    try {
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Login failed');
      }

      const data = await response.json();

      // Store token and user info in localStorage
      localStorage.setItem('token', 'mock-token'); // In real implementation, use actual token
      localStorage.setItem('userEmail', email);

      // Fetch user profile after login
      await fetchProfile(email);
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const signup = async (email: string, password: string, software_experience: string, hardware_experience: string) => {
    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          software_experience,
          hardware_experience
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Signup failed');
      }

      const data = await response.json();

      // Store token and user info in localStorage
      localStorage.setItem('token', 'mock-token'); // In real implementation, use actual token
      localStorage.setItem('userEmail', email);

      // Set the user in state
      const newUser: User = {
        id: data.user_id,
        email,
        software_experience,
        hardware_experience
      };
      setUser(newUser);
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  };

  const logout = () => {
    localStorage.removeItem('token');
    localStorage.removeItem('userEmail');
    setUser(null);
  };

  const fetchProfile = async (email: string) => {
    try {
      const response = await fetch(`/api/auth/profile?email=${encodeURIComponent(email)}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('token')}` // In real implementation
        },
      });

      if (!response.ok) {
        if (response.status === 404) {
          // Profile doesn't exist, which is okay
          return;
        }
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to fetch profile');
      }

      const profileData = await response.json();

      const userProfile: User = {
        id: profileData.user_id,
        email: profileData.email,
        software_experience: profileData.software_experience,
        hardware_experience: profileData.hardware_experience
      };

      setUser(userProfile);
    } catch (error) {
      console.error('Fetch profile error:', error);
      throw error;
    }
  };

  const updateProfile = async (software_experience?: string, hardware_experience?: string) => {
    if (!user) {
      throw new Error('No user is logged in');
    }

    try {
      const response = await fetch(`/api/auth/profile?email=${encodeURIComponent(user.email)}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('token')}` // In real implementation
        },
        body: JSON.stringify({
          software_experience,
          hardware_experience
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to update profile');
      }

      const data = await response.json();

      // Update user in state
      if (software_experience) {
        setUser(prev => prev ? { ...prev, software_experience } : null);
      }
      if (hardware_experience) {
        setUser(prev => prev ? { ...prev, hardware_experience } : null);
      }
    } catch (error) {
      console.error('Update profile error:', error);
      throw error;
    }
  };

  const value = {
    user,
    loading,
    login,
    logout,
    signup,
    updateProfile,
    fetchProfile
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};