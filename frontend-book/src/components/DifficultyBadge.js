import React from 'react';
import clsx from 'clsx';
import styles from './DifficultyBadge.module.css';

const DifficultyBadge = ({ level, size = 'medium' }) => {
  const getDifficultyInfo = (level) => {
    switch (level.toLowerCase()) {
      case 'beginner':
        return { text: 'Beginner', color: 'var(--ifm-color-success)', bg: 'rgba(16, 185, 129, 0.1)' };
      case 'intermediate':
        return { text: 'Intermediate', color: 'var(--ifm-color-warning)', bg: 'rgba(245, 158, 11, 0.1)' };
      case 'advanced':
        return { text: 'Advanced', color: 'var(--ifm-color-danger)', bg: 'rgba(239, 68, 68, 0.1)' };
      default:
        return { text: level, color: 'var(--ifm-color-secondary)', bg: 'rgba(100, 116, 139, 0.1)' };
    }
  };

  const { text, color, bg } = getDifficultyInfo(level);

  return (
    <span
      className={clsx(styles.difficultyBadge, styles[size])}
      style={{
        backgroundColor: bg,
        color: color,
        border: `1px solid ${color}40` // 40 is hex for ~25% opacity
      }}
    >
      {text}
    </span>
  );
};

export default DifficultyBadge;