import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { useAuth } from '../../contexts/AuthContext';

interface ChapterContentProps {
  chapterId: string;
  defaultContent: string | React.ReactNode;
  className?: string;
}

const ChapterContent: React.FC<ChapterContentProps> = ({
  chapterId,
  defaultContent,
  className = ''
}) => {
  const { user } = useAuth();
  const { personalizationStatus } = usePersonalization();
  const [content, setContent] = useState<React.ReactNode>(defaultContent);
  const [isPersonalized, setIsPersonalized] = useState(false);

  useEffect(() => {
    const loadContent = async () => {
      if (user && personalizationStatus[chapterId]) {
        setIsPersonalized(personalizationStatus[chapterId].isPersonalized);

        if (personalizationStatus[chapterId].isPersonalized) {
          // In a real implementation, we would fetch personalized content from the backend
          // For now, we'll just apply basic personalization based on user preferences
          const personalizedContent = applyPersonalization(defaultContent, personalizationStatus[chapterId].preferences);
          setContent(personalizedContent);
        } else {
          setContent(defaultContent);
        }
      } else {
        setContent(defaultContent);
        setIsPersonalized(false);
      }
    };

    loadContent();
  }, [chapterId, defaultContent, user, personalizationStatus]);

  // Helper function to apply basic personalization
  const applyPersonalization = (content: React.ReactNode, preferences: any) => {
    if (!preferences) return content;

    // Apply visual styling based on preferences
    const personalizedStyle: React.CSSProperties = {};

    if (preferences.theme === 'dark') {
      personalizedStyle.backgroundColor = '#1a1a1a';
      personalizedStyle.color = '#ffffff';
      personalizedStyle.padding = '16px';
      personalizedStyle.borderRadius = '8px';
    } else if (preferences.theme === 'light') {
      personalizedStyle.backgroundColor = '#f8f9fa';
      personalizedStyle.color = '#000000';
      personalizedStyle.padding = '16px';
      personalizedStyle.borderRadius = '8px';
    }

    if (preferences.fontSize === 'large') {
      personalizedStyle.fontSize = '1.2em';
    } else if (preferences.fontSize === 'small') {
      personalizedStyle.fontSize = '0.9em';
    }

    return (
      <div
        className="personalized-content"
        style={personalizedStyle}
        data-personalized={true}
      >
        {content}
      </div>
    );
  };

  return (
    <div className={`chapter-content ${className} ${isPersonalized ? 'personalized' : 'default'}`}>
      {content}
      {isPersonalized && (
        <div className="personalization-indicator">
          <small style={{
            display: 'block',
            marginTop: '10px',
            fontStyle: 'italic',
            color: '#2196F3'
          }}>
            Content has been personalized based on your preferences
          </small>
        </div>
      )}
    </div>
  );
};

export default ChapterContent;