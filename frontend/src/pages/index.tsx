import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import AboutSection from '@site/src/components/AboutSection';
import VisualBreathingSpace from '@site/src/components/VisualBreathingSpace';
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
                className={clsx('button button--secondary button--lg', styles.primaryButton)}
                to="/docs">
                Start Reading
              </Link>
              <Link
                className={clsx('button button--outline button--lg', styles.secondaryButton)}
                to="/docs/intro">
                Explore Chapters
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.overlappingCircles}>
              <div className={styles.circleOne}></div>
              <div className={styles.circleTwo}></div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <div style={{ position: 'relative' }}>
      <VisualBreathingSpace />
      <div style={{ position: 'relative', zIndex: 1 }}>
        <Layout
          title={`Hello from ${siteConfig.title}`}
          description="Description will go into a meta tag in <head />"
        >
          <HomepageHeader />
          <main>
            <HomepageFeatures />
            <AboutSection />
          </main>
        </Layout>
      </div>
    </div>
  );
}



