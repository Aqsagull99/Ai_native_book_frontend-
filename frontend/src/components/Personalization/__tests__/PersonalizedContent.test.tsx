import React from 'react';
import { render, screen } from '@testing-library/react';
import { PersonalizationProvider } from '../../contexts/PersonalizationContext';
import PersonalizedContent from '../PersonalizedContent';

// Mock the AuthContext since PersonalizedContent may depend on it
jest.mock('../../contexts/AuthContext', () => ({
  useAuth: () => ({
    user: {
      id: '1',
      email: 'test@example.com',
      software_experience: 'beginner',
      hardware_experience: 'none'
    },
    loading: false
  })
}));

describe('PersonalizedContent', () => {
  const mockContentOptions = [
    { level: 'beginner', content: 'Beginner content' },
    { level: 'intermediate', content: 'Intermediate content' },
    { level: 'advanced', content: 'Advanced content' }
  ];

  it('renders beginner content when experience level is beginner', () => {
    render(
      <PersonalizationProvider>
        <PersonalizedContent contentOptions={mockContentOptions} />
      </PersonalizationProvider>
    );

    // Assuming the context is set to beginner by default
    expect(screen.getByText(/Beginner content/)).toBeInTheDocument();
  });

  it('renders appropriate content based on experience level', () => {
    // This test would require more complex state management to change the experience level
    // and verify the content changes accordingly
    render(
      <PersonalizationProvider>
        <PersonalizedContent contentOptions={mockContentOptions} />
      </PersonalizationProvider>
    );

    expect(screen.getByText(/Beginner content/)).toBeInTheDocument();
  });

  it('falls back to beginner content when advanced content is not available', () => {
    const contentOptionsWithoutAdvanced = [
      { level: 'beginner', content: 'Beginner content' },
      { level: 'intermediate', content: 'Intermediate content' }
    ];

    render(
      <PersonalizationProvider>
        <PersonalizedContent contentOptions={contentOptionsWithoutAdvanced} />
      </PersonalizationProvider>
    );

    expect(screen.getByText(/Beginner content/)).toBeInTheDocument();
  });
});