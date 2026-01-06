# Quickstart Guide: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade
**Created**: 2025-12-17
**Status**: Complete

## Getting Started

### Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

### Setup Development Environment

1. **Clone the repository:**
```bash
git clone <repository-url>
cd frontend-book
```

2. **Install dependencies:**
```bash
npm install
```

3. **Start development server:**
```bash
npm start
```

Your site will be running at `http://localhost:3000` with hot reloading enabled.

### Verify Setup
- Confirm the site loads without errors
- Check that existing content displays properly
- Verify the development server rebuilds on file changes

## Theme Customization

### Implementing the Robotics Theme

1. **Update CSS variables in `src/css/custom.css`:**
```css
:root {
  /* Robotics-themed dark mode colors */
  --ifm-color-primary: #0ea5e9; /* blue-500 */
  --ifm-color-primary-dark: #0284c7; /* blue-600 */
  --ifm-color-primary-darker: #0369a1; /* blue-700 */
  --ifm-color-primary-darkest: #075985; /* blue-800 */
  --ifm-color-primary-light: #38bdf8; /* blue-400 */
  --ifm-color-primary-lighter: #7dd3fc; /* blue-300 */
  --ifm-color-primary-lightest: #bae6fd; /* blue-200 */

  /* Module-specific accent colors */
  --module-accent-1: #8b5cf6; /* purple */
  --module-accent-2: #ec4899; /* pink */
  --module-accent-3: #f59e0b; /* amber */
  --module-accent-4: #10b981; /* emerald */

  /* Metallic accents */
  --metallic-gold: #fbbf24;
  --metallic-silver: #e5e7eb;
  --metallic-bronze: #f97316;

  /* Dark mode background */
  --ifm-background-color: #0f172a; /* slate-900 */
  --ifm-background-surface-color: #1e293b; /* slate-800 */

  /* Text colors */
  --ifm-font-color-base: #f1f5f9; /* slate-100 */
  --ifm-font-color-base-inverse: #0f172a; /* slate-900 */
}

/* Light mode variables (if needed) */
[data-theme='light'] {
  --ifm-color-primary: #0ea5e9; /* blue-500 */
  --ifm-background-color: #ffffff;
  --ifm-font-color-base: #1e293b; /* slate-800 */
}
```

2. **Update typography in `docusaurus.config.js`:**
```javascript
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    fonts: {
      font: ['Inter', 'sans-serif'],
      monospace: ['JetBrains Mono', 'monospace']
    }
  }
};
```

3. **Add fonts via CDN in `static/css/fonts.css`:**
```css
/* Inter Font */
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');

/* JetBrains Mono Font */
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600&display=swap');

/* Apply fonts */
body {
  font-family: 'Inter', sans-serif;
}

code, pre, .code-block {
  font-family: 'JetBrains Mono', monospace;
}
```

## Implementing Key Features

### 0. Homepage Design Implementation

1. **Update homepage component in `src/pages/index.tsx`:**
```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)} style={{background: '#0a3d2e', color: 'white'}}>
      <div className="container">
        <div className={styles.heroContent}>
          <img
            src="/img/humanoid-robot.png"
            alt="Humanoid Robot"
            className={styles.robotImage}
          />
          <h1 className={clsx('hero__title', styles.mainTitle)}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--secondary button--lg', styles.primaryButton)}
              to="/docs/intro">
              Start Reading Book
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book">
      <HomepageHeader />
    </Layout>
  );
}
```

2. **Update homepage CSS in `src/pages/index.module.css`:**
```css
/**
 * CSS files with the .module.css suffix will be treated as CSS modules
 * and scoped locally.
 */

.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: #0a3d2e !important; /* Dark green background */
  color: white;
  min-height: 80vh;
  display: flex;
  align-items: center;
}

@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem;
  }
}

.heroContent {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}

.robotImage {
  max-width: 300px;
  max-height: 300px;
  margin-bottom: 2rem;
  border-radius: 8px;
}

.mainTitle {
  color: white !important;
  font-size: 3rem;
  margin-bottom: 1rem;
  text-align: center;
}

@media screen and (max-width: 996px) {
  .mainTitle {
    font-size: 2rem;
  }
}

.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
  margin-top: 2rem;
}

.primaryButton {
  background-color: #4ade80 !important; /* Green color */
  border-color: #4ade80 !important;
  color: #1f2937 !important; /* Dark text for contrast */
  font-size: 1.2rem;
  padding: 1rem 2rem;
}

.primaryButton:hover {
  background-color: #22c55e !important; /* Darker green on hover */
  border-color: #22c55e !important;
}

@media screen and (max-width: 996px) {
  .robotImage {
    max-width: 200px;
    max-height: 200px;
  }

  .heroContent {
    padding: 0 1rem;
  }
}
```

### 1. Sticky Sidebar with Progress Tracking

1. **Create progress tracking service in `src/utils/progress.js`:**
```javascript
// Client-side progress tracking using localStorage
class ProgressTracker {
  constructor() {
    this.storageKey = 'docusaurus-progress';
  }

  getProgress() {
    try {
      const data = localStorage.getItem(this.storageKey);
      return data ? JSON.parse(data) : this.getDefaultProgress();
    } catch (error) {
      console.error('Error reading progress data:', error);
      return this.getDefaultProgress();
    }
  }

  updateProgress(moduleId, sectionId, completed) {
    const progress = this.getProgress();
    if (!progress.modules[moduleId]) {
      progress.modules[moduleId] = { sections: {} };
    }

    const now = new Date().toISOString();
    progress.modules[moduleId].sections[sectionId] = {
      completed,
      startedAt: now,
      completedAt: completed ? now : null,
      timeSpent: 0
    };

    try {
      localStorage.setItem(this.storageKey, JSON.stringify(progress));
      return true;
    } catch (error) {
      console.error('Error saving progress data:', error);
      return false;
    }
  }

  getDefaultProgress() {
    return {
      modules: {},
      lastAccessed: new Date().toISOString(),
      preferences: {
        theme: 'dark',
        accentColor: 'module1'
      }
    };
  }
}

export default new ProgressTracker();
```

2. **Create sticky sidebar component in `src/components/StickySidebar.js`:**
```javascript
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
  const totalSections = sections.length;
  const progressPercentage = totalSections > 0 ? Math.round((completedSections / totalSections) * 100) : 0;

  return (
    <div className="sticky-sidebar">
      <div className="progress-bar">
        <div className="progress-text">{progressPercentage}% Complete</div>
        <div className="progress-fill" style={{ width: `${progressPercentage}%` }}></div>
      </div>

      <nav className="sidebar-nav">
        <ul>
          {sections.map((section) => (
            <li
              key={section.id}
              className={`nav-item ${progress.sections[section.id]?.completed ? 'completed' : ''}`}
            >
              <a
                href={section.url}
                onClick={(e) => {
                  if (!progress.sections[section.id]?.completed) {
                    updateSectionProgress(section.id, true);
                  }
                }}
              >
                {section.title}
                {progress.sections[section.id]?.completed && <span className="completed-badge">✓</span>}
              </a>
            </li>
          ))}
        </ul>
      </nav>
    </div>
  );
};

export default StickySidebar;
```

### 2. Enhanced Search with Cmd/Ctrl+K

1. **Create search component in `src/components/EnhancedSearch.js`:**
```javascript
import React, { useState, useEffect, useRef } from 'react';

const EnhancedSearch = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const searchRef = useRef(null);

  // Handle keyboard shortcut
  useEffect(() => {
    const handleKeyDown = (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        setIsOpen(true);
      }
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen]);

  // Focus search input when opened
  useEffect(() => {
    if (isOpen && searchRef.current) {
      searchRef.current.focus();
    }
  }, [isOpen]);

  const handleSearch = async (searchQuery) => {
    // In a real implementation, this would call your search API
    // For now, we'll simulate search results
    setQuery(searchQuery);
    if (searchQuery.length > 2) {
      // Simulate API call delay
      setTimeout(() => {
        // This would be replaced with actual search API call
        setResults([
          { id: 1, title: `Search results for: ${searchQuery}`, url: '#', moduleId: 'module1' },
          { id: 2, title: `Related to: ${searchQuery}`, url: '#', moduleId: 'module2' }
        ]);
      }, 300);
    } else {
      setResults([]);
    }
  };

  return (
    <>
      <div className={`search-modal ${isOpen ? 'open' : ''}`}>
        <div className="search-container">
          <div className="search-input-container">
            <input
              ref={searchRef}
              type="text"
              placeholder="Search documentation (Cmd/Ctrl + K)"
              value={query}
              onChange={(e) => handleSearch(e.target.value)}
              className="search-input"
            />
            <button
              className="close-button"
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>

          <div className="search-results">
            {results.map(result => (
              <a key={result.id} href={result.url} className="search-result">
                <div className="result-title">{result.title}</div>
                <div className="result-module">Module: {result.moduleId}</div>
              </a>
            ))}
          </div>
        </div>
      </div>

      <button
        className="search-shortcut-button"
        onClick={() => setIsOpen(true)}
        title="Search (Cmd/Ctrl + K)"
      >
        ⌘K
      </button>
    </>
  );
};

export default EnhancedSearch;
```

### 3. Enhanced Code Blocks

1. **Create enhanced code block component in `src/components/EnhancedCodeBlock.js`:**
```javascript
import React, { useState } from 'react';
import CopyButton from './CopyButton';

const EnhancedCodeBlock = ({ children, className = '', title, showLineNumbers = false }) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(children.trim());
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy text: ', err);
    }
  };

  return (
    <div className={`code-block-container ${className}`}>
      {title && <div className="code-block-title">{title}</div>}
      <div className="code-block-header">
        <div className="code-block-actions">
          <CopyButton
            copied={copied}
            onCopy={handleCopy}
            code={children.trim()}
          />
        </div>
      </div>
      <pre className="code-block-pre">
        <code className={className.replace('code-block-container ', '')}>
          {children}
        </code>
      </pre>
    </div>
  );
};

export default EnhancedCodeBlock;
```

### 4. Expandable Sections

1. **Create expandable section component in `src/components/ExpandableSection.js`:**
```javascript
import React, { useState } from 'react';

const ExpandableSection = ({ title, children, defaultOpen = false }) => {
  const [isOpen, setIsOpen] = useState(defaultOpen);

  return (
    <div className={`expandable-section ${isOpen ? 'open' : ''}`}>
      <button
        className="expandable-header"
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
      >
        <span className="expandable-title">{title}</span>
        <span className={`expandable-icon ${isOpen ? 'expanded' : ''}`}>
          {isOpen ? '▼' : '▶'}
        </span>
      </button>

      {isOpen && (
        <div className="expandable-content">
          {children}
        </div>
      )}
    </div>
  );
};

export default ExpandableSection;
```

### 5. Image Zoom Functionality

1. **Create image zoom component in `src/components/ImageZoom.js`:**
```javascript
import React, { useState } from 'react';

const ImageZoom = ({ src, alt, caption, ...props }) => {
  const [isZoomed, setIsZoomed] = useState(false);
  const [zoomLevel, setZoomLevel] = useState(1);

  const handleZoom = () => {
    setIsZoomed(!isZoomed);
    setZoomLevel(isZoomed ? 1 : 2);
  };

  const handleWheel = (e) => {
    if (isZoomed) {
      e.preventDefault();
      const delta = e.deltaY > 0 ? -0.1 : 0.1;
      const newZoom = Math.max(1, Math.min(4, zoomLevel + delta));
      setZoomLevel(newZoom);
    }
  };

  return (
    <div className="image-zoom-container">
      <div
        className={`zoomable-image ${isZoomed ? 'zoomed' : ''}`}
        onClick={handleZoom}
        onWheel={handleWheel}
      >
        <img
          src={src}
          alt={alt}
          style={{ transform: `scale(${zoomLevel})`, cursor: 'zoom-in' }}
          {...props}
        />
        {isZoomed && (
          <div className="zoom-overlay" onClick={() => setIsZoomed(false)}>
            <img
              src={src}
              alt={alt}
              style={{
                transform: `scale(${zoomLevel})`,
                maxWidth: '90vw',
                maxHeight: '90vh'
              }}
            />
          </div>
        )}
      </div>
      {caption && <div className="image-caption">{caption}</div>}
    </div>
  );
};

export default ImageZoom;
```

## Deployment

### Build for Production
```bash
npm run build
```

### Deploy to GitHub Pages
1. Ensure GitHub Pages is enabled in your repository settings
2. The build command generates static files in the `build/` directory
3. These files can be deployed directly to GitHub Pages

### Verify Deployment
- Check that all features work in the deployed environment
- Verify that progress tracking persists across sessions
- Confirm all UI enhancements display correctly
- Test mobile responsiveness

## Troubleshooting

### Common Issues
1. **CSS variables not applying**: Ensure custom CSS is loaded after Docusaurus default styles
2. **Search not working**: Check that search API endpoints are properly configured
3. **Progress not saving**: Verify localStorage permissions and browser support

### Performance Optimization
- Minimize JavaScript bundle size
- Optimize images and other assets
- Use CSS containment for animations
- Implement lazy loading for off-screen content