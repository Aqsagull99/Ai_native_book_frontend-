import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import PersonalizationButton from './PersonalizationButton';

interface ChapterPersonalizationWrapperProps {
  children: React.ReactNode;
  chapterId: string;
  chapterTitle?: string;
}

const ChapterPersonalizationWrapper: React.FC<ChapterPersonalizationWrapperProps> = ({
  children,
  chapterId,
  chapterTitle = 'this chapter'
}) => {
  const { user } = useAuth();

  return (
    <div className="chapter-personalization-wrapper">
      {user && (
        <div className="personalization-button-container margin-bottom--lg">
          <PersonalizationButton
            chapterId={chapterId}
            chapterTitle={chapterTitle}
          />
        </div>
      )}
      <div className="chapter-content">
        {children}
      </div>
    </div>
  );
};

export default ChapterPersonalizationWrapper;