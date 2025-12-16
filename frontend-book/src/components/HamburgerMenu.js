import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './HamburgerMenu.module.css';

const HamburgerMenu = ({ items, logo, title }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMobile, setIsMobile] = useState(false);

  // Check if we're on mobile view
  useEffect(() => {
    const checkMobile = () => {
      setIsMobile(window.innerWidth < 996);
    };

    checkMobile();
    window.addEventListener('resize', checkMobile);

    return () => {
      window.removeEventListener('resize', checkMobile);
    };
  }, []);

  // Close menu when window is resized to desktop size
  useEffect(() => {
    if (!isMobile && isOpen) {
      setIsOpen(false);
    }
  }, [isMobile]);

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.hamburgerContainer}>
      <button
        className={clsx(styles.hamburgerButton, { [styles.open]: isOpen })}
        onClick={toggleMenu}
        aria-label={isOpen ? 'Close menu' : 'Open menu'}
        aria-expanded={isOpen}
        type="button"
      >
        <span className={styles.hamburgerLine}></span>
        <span className={styles.hamburgerLine}></span>
        <span className={styles.hamburgerLine}></span>
      </button>

      {isOpen && (
        <div className={styles.hamburgerMenu}>
          <div className={styles.menuHeader}>
            {logo && <img src={logo} alt={title} className={styles.logo} />}
            {title && <h3 className={styles.menuTitle}>{title}</h3>}
          </div>

          <nav className={styles.menuNav}>
            <ul className={styles.menuList}>
              {items && items.map((item, index) => (
                <li key={index} className={styles.menuItem}>
                  <a
                    href={item.href}
                    className={styles.menuLink}
                    onClick={() => setIsOpen(false)}
                  >
                    {item.label}
                  </a>
                </li>
              ))}
            </ul>
          </nav>
        </div>
      )}
    </div>
  );
};

export default HamburgerMenu;