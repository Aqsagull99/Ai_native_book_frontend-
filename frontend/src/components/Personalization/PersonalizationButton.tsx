import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { useAuth } from '../../contexts/AuthContext';

interface PersonalizationButtonProps {
  chapterId: string;
  chapterTitle?: string;
  onPersonalize?: (success: boolean, pointsEarned?: number) => void;
  className?: string;
  style?: React.CSSProperties;
}

const PersonalizationButton: React.FC<PersonalizationButtonProps> = ({
  chapterId,
  chapterTitle = 'this chapter',
  onPersonalize,
  className = '',
  style
}) => {
  const {
    // Legacy methods (keep for backward compatibility)
    personalizationStatus,
    activatePersonalization,
    getPersonalizationStatus,
    // NEW: AI Personalization methods
    isChapterPersonalized,
    isPersonalizing,
    openPanel,
    revertToOriginal
  } = usePersonalization();
  const { user } = useAuth();

  // Use the new AI personalization state
  const isAIPersonalized = isChapterPersonalized(chapterId);

  const handleButtonClick = () => {
    console.log('PersonalizationButton clicked, chapterId:', chapterId, 'user:', !!user, 'isAIPersonalized:', isAIPersonalized);

    if (!user) {
      alert('Please sign in to personalize content');
      return;
    }

    if (isAIPersonalized) {
      // If already personalized, clicking reverts to original
      console.log('Reverting to original for chapter:', chapterId);
      revertToOriginal(chapterId);
      onPersonalize?.(true);
    } else {
      // Open the personalization panel
      console.log('Opening panel for chapter:', chapterId);
      openPanel(chapterId);
    }
  };

  // Determine button text and state
  const getButtonText = () => {
    if (isPersonalizing) {
      return 'Personalizing...';
    }
    if (isAIPersonalized) {
      return 'Revert to Original';
    }
    return 'Personalize Content';
  };

  // Define styling based on state
  const buttonStyle: React.CSSProperties = {
    backgroundColor: isAIPersonalized ? '#4CAF50' : '#2196F3', // Green when personalized, Blue otherwise
    color: 'white',
    border: 'none',
    padding: '10px 20px',
    borderRadius: '5px',
    cursor: isPersonalizing ? 'not-allowed' : 'pointer',
    fontSize: '16px',
    fontWeight: 'bold',
    transition: 'all 0.3s ease',
    opacity: isPersonalizing ? 0.7 : 1,
    display: 'flex',
    alignItems: 'center',
    gap: '8px',
    ...style
  };

  // Only show button for authenticated users
  if (!user) {
    return null;
  }

  return (
    <button
      className={`personalization-button ${className}`}
      style={buttonStyle}
      onClick={handleButtonClick}
      disabled={isPersonalizing}
      title={isAIPersonalized
        ? 'Click to revert to original content'
        : `Personalize ${chapterTitle} based on your preferences`
      }
    >
      {isPersonalizing && (
        <span style={{
          width: '16px',
          height: '16px',
          border: '2px solid rgba(255,255,255,0.3)',
          borderTopColor: 'white',
          borderRadius: '50%',
          animation: 'spin 0.8s linear infinite'
        }} />
      )}
      <span>{getButtonText()}</span>
      {isAIPersonalized && !isPersonalizing && <span>↩</span>}
    </button>
  );
};

export default PersonalizationButton;
