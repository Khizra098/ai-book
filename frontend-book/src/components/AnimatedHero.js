import React from 'react';
import clsx from 'clsx';
import styles from './AnimatedHero.module.css';

const AnimatedHero = ({ title, subtitle, actions }) => {
  return (
    <div className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={clsx('hero__title', styles.animatedTitle)}>
            {title}
          </h1>
          <p className={clsx('hero__subtitle', styles.animatedSubtitle)}>
            {subtitle}
          </p>
          {actions && (
            <div className={clsx('hero__project-tagline', styles.animatedActions)}>
              {actions.map((action, index) => (
                <a
                  key={index}
                  className={clsx('button button--secondary button--lg', styles.animatedButton)}
                  href={action.href}
                  target={action.target || '_self'}
                >
                  {action.text}
                </a>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default AnimatedHero;