import React from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useAuth } from '../../../contexts/AuthContext';
import { useTranslation } from '../../../contexts/TranslationContext';
import { usePersonalization } from '../../../contexts/PersonalizationContext';
import PersonalizationButton from '../../../components/Personalization/PersonalizationButton';
import PersonalizationPanel from '../../../components/Personalization/PersonalizationPanel';
import ChapterContent from '../../../components/Chapter/ChapterContent';
import DocItemPaginator from '@theme/DocItem/Paginator';

// console.log('🔥 CUSTOM DocItem/Layout LOADED');

export default function DocItemLayout({ children }) {
  const { metadata, frontMatter } = useDoc();
  const { user } = useAuth();
  const { loading, error } = useTranslation();
  const { isPanelOpen, activeChapterId } = usePersonalization();

  // Debug logging
  console.log('DocItemLayout render - isPanelOpen:', isPanelOpen, 'activeChapterId:', activeChapterId, 'metadata.id:', metadata.id);

  // Extract the content from children and wrap it with ChapterContent
  // This allows the translation functionality to work with the document content
  const contentWithTranslation = (
    <ChapterContent
      chapterId={metadata.id}
      defaultContent={children}
      className="chapter-content-wrapper"
    />
  );

  return (
    <>
      {contentWithTranslation}
      <div className="margin-top--xl">
        <DocItemPaginator />
      </div>

      {/* Personalization Button - Fixed position at bottom left */}
      {user && (
        <div style={{
          position: 'fixed',
          bottom: '20px',
          left: '20px',
          zIndex: 999
        }}>
          <PersonalizationButton
            chapterId={metadata.id}
            chapterTitle={metadata.title}
          />
        </div>
      )}

      {/* Personalization Panel - rendered at root level to avoid DOM issues */}
      {isPanelOpen && activeChapterId === metadata.id && (
        <PersonalizationPanel chapterId={metadata.id} />
      )}
    </>
  );
}
