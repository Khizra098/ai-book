
import React from 'react';
import clsx from 'clsx';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function HeroSection() {
  return (
    <section className={clsx('hero hero--primary', styles.heroSection)}>
      <div className="container">
        <div className={styles.heroLogo}>🤖 Humanoid Robot</div>

        <h1 className={clsx('hero__title', styles.heroTitle)}>
          🧠 Physical AI & Humanoid Robotics
        </h1>

        <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
          ⚙️ A practical guide to building intelligent humanoid systems
        </p>

        <div className={styles.heroImageContainer}>
          <img
            src={useBaseUrl('/img/ai pic.jpg')}
            alt="Humanoid Robot"
            className={styles.heroImage}
          />
        </div>
      </div>
    </section>
  );
}

export default React.memo(HeroSection);

