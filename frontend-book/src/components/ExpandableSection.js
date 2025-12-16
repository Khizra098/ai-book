import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ExpandableSection.module.css';

const ExpandableSection = ({ title, children, defaultOpen = false }) => {
  const [isOpen, setIsOpen] = useState(defaultOpen);

  return (
    <div className={clsx(styles.expandableSection, { [styles.open]: isOpen })}>
      <button
        className={styles.expandableHeader}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        type="button"
      >
        <span className={styles.expandableTitle}>{title}</span>
        <span className={clsx(styles.expandableIcon, { [styles.expanded]: isOpen })}>
          {isOpen ? '▼' : '▶'}
        </span>
      </button>

      {isOpen && (
        <div className={styles.expandableContent}>
          {children}
        </div>
      )}
    </div>
  );
};

export default ExpandableSection;