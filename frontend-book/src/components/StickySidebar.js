import React, { useState, useEffect } from 'react';
import ProgressTracker from '../utils/progress';

const StickySidebar = ({ moduleId, sections }) => {
  const [progress, setProgress] = useState({});

  useEffect(() => {
    const currentProgress = ProgressTracker.getProgress();
    setProgress(currentProgress.modules[moduleId] || { sections: {} });
  }, [moduleId]);

  const updateSectionProgress = (sectionId, completed) => {
    ProgressTracker.updateProgress(moduleId, sectionId, completed);
    const currentProgress = ProgressTracker.getProgress();
    setProgress(currentProgress.modules[moduleId] || { sections: {} });
  };

  const completedSections = Object.values(progress.sections).filter(s => s.completed).length;
  const totalSections = sections ? sections.length : 0;
  const progressPercentage = totalSections > 0 ? Math.round((completedSections / totalSections) * 100) : 0;

  return (
    <div className="sticky-sidebar">
      <div className="progress-bar-container">
        <div className="progress-text">{progressPercentage}% Complete</div>
        <div className="progress-bar">
          <div
            className="progress-fill"
            style={{ width: `${progressPercentage}%` }}
          ></div>
        </div>
      </div>

      <nav className="sidebar-nav">
        <ul className="menu__list">
          {sections && sections.map((section) => (
            <li key={section.id} className="menu__list-item">
              <a
                href={section.url}
                className={`menu__link ${progress.sections[section.id]?.completed ? 'menu__link--active' : ''}`}
                onClick={(e) => {
                  if (!progress.sections[section.id]?.completed) {
                    updateSectionProgress(section.id, true);
                  }
                }}
              >
                {section.title}
                {progress.sections[section.id]?.completed && (
                  <span className="completed-badge" style={{
                    marginLeft: '8px',
                    color: 'var(--ifm-color-success)'
                  }}>✓</span>
                )}
              </a>
            </li>
          ))}
        </ul>
      </nav>
    </div>
  );
};

export default StickySidebar;