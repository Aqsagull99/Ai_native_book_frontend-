import React, { useState, useEffect, useMemo } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { useAuth } from '../../contexts/AuthContext';
import { useTranslation } from '../../contexts/TranslationContext';
import { translateContent } from '../../services/translationService';
import styles from './ChapterContent.module.css';

// Helper function to sanitize HTML and remove potentially harmful elements
const sanitizeHtml = (html: string): string => {
  // Remove script tags and their content
  let sanitized = html.replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '');
  // Remove onclick, onerror, onload and other event handlers
  sanitized = sanitized.replace(/\son\w+\s*=\s*["'][^"']*["']/gi, '');
  // Remove javascript: URLs
  sanitized = sanitized.replace(/href\s*=\s*["']javascript:[^"']*["']/gi, 'href="#"');
  // Remove style tags that might cause issues
  sanitized = sanitized.replace(/<style\b[^<]*(?:(?!<\/style>)<[^<]*)*<\/style>/gi, '');
  return sanitized;
};

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
  const {
    personalizationStatus,
    // NEW: AI Personalization
    getContentForChapter,
    isChapterPersonalized,
    isPersonalizing,
    activeChapterId
  } = usePersonalization();
  const { isTranslated, translateToUrdu, revertToOriginal: revertTranslation, loading, error } = useTranslation();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslationButtonVisible, setIsTranslationButtonVisible] = useState(false);

  // NEW: Get AI personalized content for this chapter
  const aiPersonalizedEntry = getContentForChapter(chapterId);
  const isAIPersonalized = isChapterPersonalized(chapterId);

  // Check if we're showing translated content
  const showingTranslation = isTranslated[chapterId] && isTranslated[chapterId].translatedContent;
  const translatedContent = showingTranslation ? isTranslated[chapterId].translatedContent : null;

  useEffect(() => {
    // Check if user is authenticated to show translation button
    setIsTranslationButtonVisible(!!user);

    if (user && personalizationStatus[chapterId]) {
      setIsPersonalized(personalizationStatus[chapterId].isPersonalized);
    } else {
      setIsPersonalized(false);
    }
  }, [chapterId, user, personalizationStatus]);

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

  const handleTranslateClick = async () => {
    if (!user) {
      alert('Please log in to use the translation feature');
      return;
    }

    try {
      // Extract text content from the DOM for translation
      // This gets the actual rendered text including all markdown/HTML content
      const contentElement = document.querySelector('.chapter-content');
      let textContent = '';

      if (contentElement) {
        // Get the inner HTML to preserve structure for translation
        textContent = contentElement.innerHTML;
      } else if (typeof defaultContent === 'string') {
        textContent = defaultContent;
      } else {
        // Fallback: try to get text from React node
        textContent = '';
      }

      if (!textContent || textContent.trim() === '') {
        console.error('No content found to translate');
        return;
      }

      console.log('Translating content length:', textContent.length);
      await translateToUrdu(chapterId, textContent);
    } catch (err) {
      console.error('Translation error:', err);
    }
  };

  const handleRevertClick = () => {
    revertTranslation(chapterId);
  };

  // Determine what content to display
  const renderContent = () => {
    // Priority: Translation > AI Personalization > Original

    // If translation is active, show translated content
    if (showingTranslation) {
      return (
        <div
          className="translated-content"
          dir="rtl"
          lang="ur"
          dangerouslySetInnerHTML={{ __html: translatedContent as string }}
        />
      );
    }

    // NEW: If AI personalized, show personalized content (sanitized for safety)
    if (isAIPersonalized && aiPersonalizedEntry?.content) {
      const sanitizedContent = sanitizeHtml(aiPersonalizedEntry.content);
      console.log('Rendering personalized content, length:', sanitizedContent.length);
      return (
        <div
          className="ai-personalized-content"
          dangerouslySetInnerHTML={{ __html: sanitizedContent }}
        />
      );
    }

    // Otherwise, show original content
    return defaultContent;
  };

  // Loading overlay during personalization
  const renderLoadingOverlay = () => {
    if (isPersonalizing && activeChapterId === chapterId) {
      return (
        <div className={styles.loadingOverlay || 'loading-overlay'} style={{
          position: 'absolute',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(255, 255, 255, 0.8)',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 10
        }}>
          <div style={{
            width: '40px',
            height: '40px',
            border: '4px solid #e0e0e0',
            borderTopColor: '#2196F3',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite'
          }} />
          <p style={{ marginTop: '16px', color: '#666' }}>Personalizing content...</p>
        </div>
      );
    }
    return null;
  };

  return (
    <div
      className={`chapter-content ${className} ${isPersonalized ? 'personalized' : 'default'} ${isAIPersonalized ? 'ai-personalized' : ''}`}
      style={{ position: 'relative' }}
    >
      {/* Translation button - visible only for logged-in users */}
      {isTranslationButtonVisible && (
        <div className={styles['translation-controls']}>
          {!isTranslated[chapterId] ? (
            <button
              className={styles['translate-button']}
              onClick={handleTranslateClick}
              disabled={loading}
              aria-label="Translate to Urdu"
            >
              {loading ? 'Translating...' : 'Translate to Urdu'}
            </button>
          ) : (
            <button
              className={styles['revert-button']}
              onClick={handleRevertClick}
              aria-label="Revert to original language"
            >
              Revert to Original
            </button>
          )}

          {loading && (
            <span className={styles['loading-indicator']}>Translating content...</span>
          )}

          {error && (
            <span className={styles['error-message']}>{error}</span>
          )}
        </div>
      )}

      {/* Loading overlay during AI personalization */}
      {renderLoadingOverlay()}

      {/* Main content area */}
      {renderContent()}

      {/* AI Personalization indicator */}
      {isAIPersonalized && aiPersonalizedEntry && (
        <div className="ai-personalization-indicator" style={{
          display: 'flex',
          alignItems: 'center',
          gap: '8px',
          marginTop: '16px',
          padding: '8px 12px',
          backgroundColor: '#e3f2fd',
          borderRadius: '4px',
          fontSize: '0.875rem',
          color: '#1976d2'
        }}>
          <span>✨</span>
          <span>
            Personalized for {aiPersonalizedEntry.preferencesUsed.readingLevel} level
            {aiPersonalizedEntry.preferencesUsed.technicalExplanations && ' • Technical explanations enabled'}
          </span>
        </div>
      )}

      {/* Legacy personalization indicator */}
      {isPersonalized && !isAIPersonalized && (
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
