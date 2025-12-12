import React from 'react';
import { useAuth } from '../../contexts/AuthContext';

interface PersonalizeContentButtonProps {
  onPersonalize: (experienceLevel: string) => void;
}

const PersonalizeContentButton: React.FC<PersonalizeContentButtonProps> = ({ onPersonalize }) => {
  const { user } = useAuth();

  const handleClick = () => {
    if (!user) {
      alert('Please sign in to personalize content');
      return;
    }

    // Determine experience level based on user profile
    let experienceLevel = 'beginner';

    // Simple logic to determine experience level
    if (user.software_experience === 'advanced' || user.hardware_experience === 'advanced') {
      experienceLevel = 'advanced';
    } else if (user.software_experience === 'intermediate' || user.hardware_experience === 'basic') {
      experienceLevel = 'intermediate';
    }

    onPersonalize(experienceLevel);
  };

  return (
    <button
      className="personalize-content-button button button--primary"
      onClick={handleClick}
    >
      Personalize Content
    </button>
  );
};

export default PersonalizeContentButton;