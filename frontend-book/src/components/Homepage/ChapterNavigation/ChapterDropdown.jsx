import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';
import { chapterData } from '@site/src/config/homepage-config';

function ChapterDropdown() {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef(null);

  // Use chapters data from configuration
  const chapters = chapterData;

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const toggleDropdown = () => {
    setIsOpen(!isOpen);
  };

  return (
    <section className={styles.chapterNavigation}>
      <div className="container">
        <div className={styles.chapterNavContainer} ref={dropdownRef}>
          <Link
            className={clsx(styles.startReadingButton, 'button button--primary')}
            to="/docs/intro"
          >
          Explore
          </Link>
          <button
            className={styles.dropdownToggle}
            onClick={toggleDropdown}
            aria-expanded={isOpen}
            aria-haspopup="true"
            aria-label="Toggle chapter navigation"
            type="button"
          >
           ⤵
          </button>
          {isOpen && (
            <div className={clsx(styles.dropdownMenu, styles.open)}>
              {chapters.map((chapter) => (
                <Link
                  key={chapter.id}
                  className={styles.dropdownItem}
                  to={chapter.url}
                  onClick={() => setIsOpen(false)}
                >
                  {chapter.title}
                </Link>
              ))}
            </div>
          )}
        </div>
      </div>
    </section>
  );
}

export default React.memo(ChapterDropdown);