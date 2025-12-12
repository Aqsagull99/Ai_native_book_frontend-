import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

interface ContentOption {
  level: 'beginner' | 'intermediate' | 'advanced';
  content: string | React.ReactNode;
}

interface PersonalizedContentProps {
  contentOptions: ContentOption[];
  className?: string;
}

const PersonalizedContent: React.FC<PersonalizedContentProps> = ({ contentOptions, className = '' }) => {
  const { getPersonalizedContent } = usePersonalization();

  const content = getPersonalizedContent(contentOptions);

  return (
    <div className={className}>
      {typeof content === 'string' ? <div dangerouslySetInnerHTML={{ __html: content }} /> : content}
    </div>
  );
};

export default PersonalizedContent;