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
    personalizationStatus,
    activatePersonalization,
    getPersonalizationStatus
  } = usePersonalization();
  const { user } = useAuth();
  const [isProcessing, setIsProcessing] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [buttonText, setButtonText] = useState('Personalize Content');

  // Check personalization status on component mount and when chapterId changes
  useEffect(() => {
    const checkStatus = async () => {
      if (user) {
        const status = await getPersonalizationStatus(chapterId);
        setIsPersonalized(status.isPersonalized);
        setButtonText(status.isPersonalized ? 'Content Personalized!' : 'Personalize Content');
      }
    };

    checkStatus();
  }, [chapterId, user, getPersonalizationStatus]);

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please sign in to personalize content and earn bonus points');
      return;
    }

    if (isPersonalized) {
      alert('Content already personalized! You have already earned your bonus points for this chapter.');
      onPersonalize?.(true);
      return;
    }

    setIsProcessing(true);

    try {
      const result = await activatePersonalization(chapterId, {
        // Default preferences for this chapter
        chapterId,
        personalizationType: 'content',
        timestamp: new Date().toISOString()
      });

      if (result) {
        setIsPersonalized(true);
        setButtonText('Content Personalized!');
        // Get the actual points earned from the personalization status
        const status = personalizationStatus[chapterId];
        onPersonalize?.(true, status?.pointsEarned || 50); // Default to 50 if not available
      } else {
        alert('Failed to personalize content. Please try again.');
        onPersonalize?.(false);
      }
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('An error occurred while personalizing content. Please try again.');
      onPersonalize?.(false);
    } finally {
      setIsProcessing(false);
    }
  };

  // Define distinctive styling as specified in requirements (orange or teal color)
  const buttonStyle: React.CSSProperties = {
    backgroundColor: '#2196F3', // Blue color as an alternative to orange/teal that contrasts well
    color: 'white',
    border: 'none',
    padding: '10px 20px',
    borderRadius: '5px',
    cursor: isProcessing || isPersonalized ? 'default' : 'pointer',
    fontSize: '16px',
    fontWeight: 'bold',
    transition: 'all 0.3s ease',
    opacity: isProcessing ? 0.7 : 1,
    pointerEvents: isProcessing || isPersonalized ? 'none' : 'auto',
    ...style
  };

  // If already personalized, show a different style
  if (isPersonalized) {
    buttonStyle.backgroundColor = '#4CAF50'; // Green for already personalized
  }

  return (
    <button
      className={`personalization-button ${className}`}
      style={buttonStyle}
      onClick={handlePersonalize}
      disabled={isProcessing || isPersonalized}
      title={`Personalize ${chapterTitle} and earn bonus points`}
    >
      {isProcessing ? 'Processing...' : buttonText}
      {isPersonalized && <span style={{ marginLeft: '8px' }}>✓</span>}
    </button>
  );
};

export default PersonalizationButton;