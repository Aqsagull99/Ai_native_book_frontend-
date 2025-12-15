import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import PersonalizationButton from '../components/Personalization/PersonalizationButton';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContainer}>
          <div className={styles.heroContent}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className={clsx('button button--secondary button--lg', styles.readBookButton)}
                to="/docs">
                Read the Book
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src="/img/robotics-hero-image.svg"
              alt="AI and Robotics Visualization"
              className={styles.heroImageContent}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <div style={{ padding: '20px', textAlign: 'center' }}>
          <h2>Personalization Test</h2>
          <p>Try the personalization button below (will only show if logged in):</p>
          <PersonalizationButton
            chapterId="homepage-test"
            chapterTitle="Homepage Test Chapter"
          />
        </div>
      </main>
    </Layout>
  );
}



