import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { FiZap, FiCpu, FiCode } from 'react-icons/fi';

type FeatureItem = {
  title: string;
  icon: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  description: React.ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Easy to Use',
    icon: FiZap,
    description: (
      <>
        Build and understand robotics systems with our step-by-step approach.
        From basic ROS concepts to advanced humanoid control, our learning path
        makes complex robotics accessible through practical, hands-on examples.
      </>
    ),
  },
  {
    title: 'Focus on What Matters',
    icon: FiCpu,
    description: (
      <>
        Master the core concepts of physical AI and system architecture.
        We handle the complexity of ROS nodes, message passing, and hardware
        abstraction layers so you can focus on building intelligent robotic systems.
      </>
    ),
  },
  {
    title: 'Powered by React',
    icon: FiCode,
    description: (
      <>
        Leverage modern web technologies to build interactive robotics applications.
        Our framework combines React's component architecture with robotics
        simulation tools for seamless development and visualization.
      </>
    ),
  },
];

function Feature({ title, icon: Icon, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureItem}>
        <div className={styles.featureIcon}>
          <Icon size={64} />
        </div>
        <div className={styles.featureContent}>
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}