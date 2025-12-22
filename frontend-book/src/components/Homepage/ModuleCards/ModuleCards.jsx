import React from 'react';
import clsx from 'clsx';
import ModuleCard from './ModuleCard';
import styles from './styles.module.css';
import { moduleData } from '@site/src/config/homepage-config';

function ModuleCards() {
  // Use modules data from configuration
  const modules = moduleData;

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className={styles.modulesHeading}>Start Book Reading</h2>
        <div className={styles.modulesGrid}>
          {modules.map((module) => (
            <ModuleCard
              key={module.id}
              title={module.title}
              description={module.description}
              link={module.link}
              buttonText={module.buttonText}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

export default React.memo(ModuleCards);