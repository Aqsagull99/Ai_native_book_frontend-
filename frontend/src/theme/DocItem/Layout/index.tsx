import React from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useAuth } from '../../../contexts/AuthContext';
import PersonalizationButton from '../../../components/Personalization/PersonalizationButton';
import DocItemPaginator from '@theme/DocItem/Paginator';

// console.log('🔥 CUSTOM DocItem/Layout LOADED');

export default function DocItemLayout({ children }) {
  const { metadata, frontMatter } = useDoc();
  const { user } = useAuth();

  return (
    <>
      {user && (
        <div className="margin-bottom--lg">
          <PersonalizationButton
            chapterId={metadata.id}
            chapterTitle={metadata.title}
          />
        </div>
      )}
      {children}
      <div className="margin-top--xl">
        <DocItemPaginator />
      </div>
    </>
  );
}
