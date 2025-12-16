import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './EnhancedSearch.module.css';

const EnhancedSearch = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const searchRef = useRef(null);
  const searchContainerRef = useRef(null);

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

  // Focus search input when opened and handle click outside
  useEffect(() => {
    if (isOpen && searchRef.current) {
      searchRef.current.focus();
    }

    const handleClickOutside = (event) => {
      if (searchContainerRef.current && !searchContainerRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
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
      {isOpen && (
        <div className={styles.searchOverlay} onClick={() => setIsOpen(false)} />
      )}
      <div className={styles.searchContainer} ref={searchContainerRef}>
        <div className={clsx(styles.searchModal, { [styles.open]: isOpen })}>
          <div className={styles.searchInputContainer}>
            <input
              ref={searchRef}
              type="text"
              placeholder="Search documentation (Cmd/Ctrl + K)"
              value={query}
              onChange={(e) => handleSearch(e.target.value)}
              className={styles.searchInput}
              onClick={(e) => e.stopPropagation()}
            />
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>

          <div className={styles.searchResults}>
            {results.map(result => (
              <a key={result.id} href={result.url} className={styles.searchResult}>
                <div className={styles.resultTitle}>{result.title}</div>
                <div className={styles.resultModule}>Module: {result.moduleId}</div>
              </a>
            ))}
          </div>
        </div>

        <button
          className={styles.searchShortcutButton}
          onClick={() => setIsOpen(true)}
          title="Search (Cmd/Ctrl + K)"
          type="button"
        >
          ⌘K
        </button>
      </div>
    </>
  );
};

export default EnhancedSearch;