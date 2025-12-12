import React, { createContext, useContext, useState, ReactNode } from 'react';

type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';

interface PersonalizationContextType {
  experienceLevel: ExperienceLevel;
  setExperienceLevel: (level: ExperienceLevel) => void;
  getPersonalizedContent: (content: { level: ExperienceLevel; content: string }[]) => string;
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

interface PersonalizationProviderProps {
  children: ReactNode;
}

export const PersonalizationProvider: React.FC<PersonalizationProviderProps> = ({ children }) => {
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel>('beginner');

  const getPersonalizedContent = (contentOptions: { level: ExperienceLevel; content: string }[]) => {
    // First, try to get content for the exact experience level
    const exactMatch = contentOptions.find(option => option.level === experienceLevel);
    if (exactMatch) {
      return exactMatch.content;
    }

    // If no exact match, find the closest appropriate level
    // For example, if we don't have advanced content, fall back to intermediate
    // If we don't have intermediate, fall back to beginner
    if (experienceLevel === 'advanced') {
      const intermediateMatch = contentOptions.find(option => option.level === 'intermediate');
      if (intermediateMatch) return intermediateMatch.content;
    }

    if (experienceLevel === 'advanced' || experienceLevel === 'intermediate') {
      const beginnerMatch = contentOptions.find(option => option.level === 'beginner');
      if (beginnerMatch) return beginnerMatch.content;
    }

    // If nothing matches, return the first available content
    return contentOptions[0]?.content || '';
  };

  const value = {
    experienceLevel,
    setExperienceLevel,
    getPersonalizedContent
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};