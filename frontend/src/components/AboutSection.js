import React from 'react';
import clsx from 'clsx';
import styles from './AboutSection.module.css';

// Simple SVG illustration component for the right column
const DocumentationIllustration = () => {
  return (
    <svg
      className={styles.illustrationSvg}
      viewBox="0 0 200 160"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Book representation */}
      <rect x="20" y="20" width="120" height="120" rx="8" fill="rgba(255,255,255,0.1)" stroke="rgba(255,255,255,0.3)" strokeWidth="1"/>
      <rect x="22" y="22" width="116" height="116" rx="6" fill="none" stroke="rgba(255,255,255,0.1)" strokeWidth="1"/>

      {/* Text lines on the book */}
      <rect x="40" y="40" width="80" height="6" rx="2" fill="rgba(255,255,255,0.2)"/>
      <rect x="40" y="50" width="70" height="6" rx="2" fill="rgba(255,255,255,0.2)"/>
      <rect x="40" y="60" width="60" height="6" rx="2" fill="rgba(255,255,255,0.2)"/>
      <rect x="40" y="75" width="80" height="6" rx="2" fill="rgba(255,255,255,0.2)"/>
      <rect x="40" y="85" width="70" height="6" rx="2" fill="rgba(255,255,255,0.2)"/>

      {/* Document card */}
      <rect x="140" y="60" width="40" height="30" rx="4" fill="rgba(255,255,255,0.1)" stroke="rgba(255,255,255,0.3)" strokeWidth="1"/>
      <rect x="145" y="68" width="30" height="4" rx="2" fill="rgba(255,255,255,0.2)"/>
      <rect x="145" y="76" width="20" height="3" rx="1.5" fill="rgba(255,255,255,0.2)"/>

      {/* Data flow line */}
      <path d="M140 75 L120 75" stroke="rgba(255,255,255,0.3)" strokeWidth="1" strokeDasharray="2,2"/>

      {/* Additional document card */}
      <rect x="140" y="100" width="40" height="25" rx="4" fill="rgba(255,255,255,0.1)" stroke="rgba(255,255,255,0.3)" strokeWidth="1"/>
      <rect x="145" y="108" width="30" height="4" rx="2" fill="rgba(255,255,255,0.2)"/>
    </svg>
  );
};

const AboutSection = () => {
  return (
    <section className={styles.aboutSection}>
      <div className="container">
        <div className={clsx('row', styles.aboutRow)}>
          <div className={clsx('col col--6', styles.textContent)}>
            <h2 className={styles.sectionTitle}>About This AI Book</h2>
            <p className={styles.sectionDescription}>
              This is an AI-powered interactive book experience that leverages advanced semantic search technology to provide contextually relevant answers grounded in the book's content.
            </p>
            <p className={styles.sectionDescription}>
              Our system uses vector embeddings and retrieval-augmented generation (RAG) to ensure all responses are strictly based on the book content, providing reliable and accurate information for learning and exploration.
            </p>
            <p className={styles.sectionDescription}>
              Designed for technical understanding and educational purposes, this platform offers a unique way to interact with complex content through intelligent querying and source attribution.
            </p>
          </div>
          <div className={clsx('col col--6', styles.illustrationContent)}>
            <div className={styles.illustrationContainer}>
              <DocumentationIllustration />
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default AboutSection;