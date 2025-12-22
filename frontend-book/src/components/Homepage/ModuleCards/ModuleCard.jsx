import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function ModuleCard({ title, description, link, buttonText = 'Explore' }) {
  return (
    <div className={styles.moduleCard}>
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleDescription}>{description}</p>
      <Link to={link} className={clsx(styles.moduleButton, 'button button--secondary')}>
        {buttonText}
      </Link>
    </div>
  );
}

export default React.memo(ModuleCard);