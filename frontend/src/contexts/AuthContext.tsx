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
  refreshPersonalization: () => void;
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
      // Safely handle environment variable for Docusaurus
      let BACKEND_URL = 'http://localhost:8000';
      if (typeof process !== 'undefined' && process && process.env) {
        BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
      }
      const response = await fetch(`${BACKEND_URL}/api/auth/signin`, {
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
      if (data.access_token) {
        localStorage.setItem('token', data.access_token);
      }
      localStorage.setItem('userEmail', email);

      // Set the user in state directly from the response data
      const loggedInUser: User = {
        id: data.user_id,
        email: data.email,
        software_experience: 'beginner', // Will be updated by fetchProfile
        hardware_experience: 'none'      // Will be updated by fetchProfile
      };
      setUser(loggedInUser);

      // Fetch user profile after login to get experience levels
      await fetchProfile(email);
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const signup = async (email: string, password: string, software_experience: string, hardware_experience: string) => {
    try {
      // Safely handle environment variable for Docusaurus
      let BACKEND_URL = 'http://localhost:8000';
      if (typeof process !== 'undefined' && process && process.env) {
        BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
      }
      const response = await fetch(`${BACKEND_URL}/api/auth/signup`, {
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
      if (data.access_token) {
        localStorage.setItem('token', data.access_token);
      }
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
      // Safely handle environment variable for Docusaurus
      let BACKEND_URL = 'http://localhost:8000';
      if (typeof process !== 'undefined' && process && process.env) {
        BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
      }
      const response = await fetch(`${BACKEND_URL}/api/auth/profile?email=${encodeURIComponent(email)}`, {
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

      // Update user state with profile data
      setUser(prevUser => ({
        ...prevUser,
        ...userProfile
      }));
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
      // Safely handle environment variable for Docusaurus
      let BACKEND_URL = 'http://localhost:8000';
      if (typeof process !== 'undefined' && process && process.env) {
        BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
      }
      const response = await fetch(`${BACKEND_URL}/api/auth/profile?email=${encodeURIComponent(user.email)}`, {
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

      await response.json(); // Consume the response but we don't need the data

      // Update user in state
      const updatedUser = { ...user };
      if (software_experience) {
        updatedUser.software_experience = software_experience;
      }
      if (hardware_experience) {
        updatedUser.hardware_experience = hardware_experience;
      }
      setUser(updatedUser);
    } catch (error) {
      console.error('Update profile error:', error);
      throw error;
    }
  };

  const refreshPersonalization = () => {
    // This function would be called to refresh personalization when needed
    // For now, it's a placeholder that can be called when personalization needs updating
  };

  const value = {
    user,
    loading,
    login,
    logout,
    signup,
    updateProfile,
    fetchProfile,
    refreshPersonalization
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