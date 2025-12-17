import React from 'react';
import clsx from 'clsx';
import styles from './VisualBreathingSpace.module.css';

const VisualBreathingSpace = () => {
  return (
    <section className={styles.visualBreathingSpace}>
      <div className={styles.circlesRow}>
        <div className={styles.circleContainer}>
          <div className={clsx(styles.circle, styles.circle1)}></div>
          <div className={clsx(styles.circle, styles.circle2)}></div>
          <div className={clsx(styles.circle, styles.circle3)}></div>
          <div className={clsx(styles.circle, styles.circle4)}></div>
          <div className={clsx(styles.circle, styles.circle5)}></div>
          <div className={clsx(styles.circle, styles.circle6)}></div>
          <div className={clsx(styles.circle, styles.circle7)}></div>
          <div className={clsx(styles.circle, styles.circle8)}></div>
        </div>
      </div>
    </section>
  );
};

export default VisualBreathingSpace;