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

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.heroBanner}>
      {/* Center content */}
      <div className={styles.inner}>

        <h1 className={styles.title}>
          Physical AI &amp; Humanoid Robotics
        </h1>

        <div className={styles.rule} />
         
        <div className={styles.rule} /> 

        <h2 className={styles.subtitle}>
          Foundations of Intelligence in Embodied Machines
        </h2>

         

        <p className={styles.description}>
          A rigorous exploration of how artificial intelligence operates in physical,
          human-scale systems.
        </p>

        {/* Diagram */}
        <div className={styles.diagram}>
          <div className={styles.diagramTop}>
            <div className={styles.box}>Perception</div>
            <span className={styles.arrow} />
            <div className={styles.box}>Cognition</div>
            <span className={styles.arrow} />
            <div className={styles.box}>Action</div>
          </div>

          <div className={styles.diagramBottom}>
            <div className={styles.box}>Sensors</div>
            <div className={styles.box}>Learning</div>
          </div>
        </div>

        <Link to="/docs" className={styles.cta}>
          View Table of Contents
        </Link>

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




