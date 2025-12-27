/**
 * PersonalizationPanel Component
 * A slide-in drawer panel for configuring AI personalization settings
 */
import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import styles from './PersonalizationPanel.module.css';

interface PersonalizationPanelProps {
  chapterId: string;
  onApply?: () => void;
}

type ReadingLevel = 'beginner' | 'intermediate' | 'advanced';
type ExampleDensity = 'minimal' | 'normal' | 'detailed';

const PersonalizationPanel: React.FC<PersonalizationPanelProps> = ({ chapterId, onApply }) => {
  const {
    aiPreferences,
    updateAIPreferences,
    personalizeChapter,
    isPersonalizing,
    personalizationError,
    isPanelOpen,
    closePanel
  } = usePersonalization();

  if (!isPanelOpen) {
    return null;
  }

  const handleApply = async () => {
    console.log('Apply Personalization clicked for chapter:', chapterId);

    // Get the chapter content from the DOM - try multiple selectors
    // Docusaurus uses different classes for content
    const selectors = [
      '.chapter-content-wrapper',
      '.chapter-content',
      '[class*="docItemCol"]',
      '.theme-doc-markdown',
      'article',
      '.markdown'
    ];

    let htmlContent = '';
    let foundSelector = '';

    for (const selector of selectors) {
      const element = document.querySelector(selector);
      if (element && element.innerHTML && element.innerHTML.length > 100) {
        htmlContent = element.innerHTML;
        foundSelector = selector;
        console.log(`Found content using selector: ${selector}, length: ${htmlContent.length}`);
        break;
      }
    }

    if (!htmlContent) {
      console.error('Could not extract chapter content - no valid selector found');
      alert('Could not find chapter content to personalize');
      return;
    }

    console.log('Extracting content for personalization, length:', htmlContent.length, 'using selector:', foundSelector);

    const success = await personalizeChapter(chapterId, htmlContent);
    console.log('personalizeChapter result:', success);

    if (success) {
      closePanel(); // Explicitly close panel on success
      if (onApply) {
        onApply();
      }
    }
  };

  const handleOverlayClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      closePanel();
    }
  };

  const readingLevels: { value: ReadingLevel; label: string; description: string }[] = [
    { value: 'beginner', label: 'Beginner', description: 'Simpler language' },
    { value: 'intermediate', label: 'Intermediate', description: 'Balanced' },
    { value: 'advanced', label: 'Advanced', description: 'Professional' }
  ];

  const densityOptions: { value: ExampleDensity; label: string; description: string }[] = [
    { value: 'minimal', label: 'Minimal', description: 'Essential examples only' },
    { value: 'normal', label: 'Normal', description: 'Original examples' },
    { value: 'detailed', label: 'Detailed', description: 'More examples' }
  ];

  return (
    <div className={styles.panelOverlay} onClick={handleOverlayClick}>
      <div className={styles.panel}>
        {/* Header */}
        <div className={styles.panelHeader}>
          <h3 className={styles.panelTitle}>Personalize Content</h3>
          <button
            className={styles.closeButton}
            onClick={closePanel}
            aria-label="Close panel"
          >
            ×
          </button>
        </div>

        {/* Content */}
        <div className={styles.panelContent}>
          {/* Error Message */}
          {personalizationError && (
            <div className={styles.errorMessage}>
              <span>⚠️</span>
              <span>{personalizationError}</span>
            </div>
          )}

          {/* Reading Level Section */}
          <div className={styles.section}>
            <h4 className={styles.sectionTitle}>Reading Level</h4>
            <div className={styles.readingLevelButtons}>
              {readingLevels.map((level) => (
                <button
                  key={level.value}
                  className={`${styles.levelButton} ${
                    aiPreferences.readingLevel === level.value ? styles.active : ''
                  }`}
                  onClick={() => updateAIPreferences({ readingLevel: level.value })}
                  disabled={isPersonalizing}
                >
                  <span className={styles.levelLabel}>{level.label}</span>
                  <span className={styles.levelDescription}>{level.description}</span>
                </button>
              ))}
            </div>
          </div>

          {/* Technical Terms Section */}
          <div className={styles.section}>
            <h4 className={styles.sectionTitle}>Technical Terms</h4>
            <div
              className={styles.toggleContainer}
              onClick={() =>
                !isPersonalizing &&
                updateAIPreferences({
                  technicalExplanations: !aiPreferences.technicalExplanations
                })
              }
              style={{ cursor: isPersonalizing ? 'not-allowed' : 'pointer' }}
            >
              <div
                className={`${styles.toggle} ${
                  aiPreferences.technicalExplanations ? styles.enabled : ''
                }`}
              >
                <div className={styles.toggleKnob} />
              </div>
              <span className={styles.toggleLabel}>Show inline explanations</span>
            </div>
          </div>

          {/* Example Density Section */}
          <div className={styles.section}>
            <h4 className={styles.sectionTitle}>Example Density</h4>
            <div className={styles.densityOptions}>
              {densityOptions.map((option) => (
                <div
                  key={option.value}
                  className={`${styles.densityOption} ${
                    aiPreferences.exampleDensity === option.value ? styles.active : ''
                  }`}
                  onClick={() =>
                    !isPersonalizing &&
                    updateAIPreferences({ exampleDensity: option.value })
                  }
                  style={{ cursor: isPersonalizing ? 'not-allowed' : 'pointer' }}
                >
                  <div className={styles.radioCircle} />
                  <div>
                    <span className={styles.densityLabel}>{option.label}</span>
                    <span className={styles.densityDescription}> - {option.description}</span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Footer */}
        <div className={styles.panelFooter}>
          <button
            className={styles.applyButton}
            onClick={handleApply}
            disabled={isPersonalizing}
          >
            {isPersonalizing ? (
              <>
                <div className={styles.spinner} />
                <span>Personalizing...</span>
              </>
            ) : (
              <span>Apply Personalization</span>
            )}
          </button>
        </div>
      </div>
    </div>
  );
};

export default PersonalizationPanel;
