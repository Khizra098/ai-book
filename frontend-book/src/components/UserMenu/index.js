import React, { useState, useEffect } from 'react';
import NavbarItem from '@theme/NavbarItem';
import { useLocation } from '@docusaurus/router';
import clsx from 'clsx';

const UserMenu = () => {
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const location = useLocation();

  // Check if user is logged in by checking for auth token in localStorage
  useEffect(() => {
    const token = localStorage.getItem('authToken');
    setIsLoggedIn(!!token);
  }, [location.pathname]);

  const handleLogout = () => {
    localStorage.removeItem('authToken');
    setIsLoggedIn(false);
    setShowDropdown(false);
    // Redirect to home page after logout
    window.location.href = '/';
  };

  // Toggle dropdown visibility
  const toggleDropdown = () => {
    setShowDropdown(!showDropdown);
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      const dropdown = document.getElementById('user-menu-dropdown');
      if (dropdown && !dropdown.contains(event.target)) {
        setShowDropdown(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  if (isLoggedIn) {
    return (
      <div className="navbar__item dropdown dropdown--hoverable dropdown--right" id="user-menu-dropdown">
        <a
          className="navbar__link"
          href="#"
          onClick={(e) => {
            e.preventDefault();
            toggleDropdown();
          }}
          aria-haspopup="true"
          aria-expanded={showDropdown}
        >
          <span className="navbar__user-icon">👤</span>
        </a>
        {showDropdown && (
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="/profile">
                Profile
              </a>
            </li>
            <li>
              <a className="dropdown__link" href="/settings">
                Settings
              </a>
            </li>
            <li>
              <a className="dropdown__link" href="#" onClick={handleLogout}>
                Log Out
              </a>
            </li>
          </ul>
        )}
      </div>
    );
  } else {
    return (
      <div className="navbar__item" id="user-menu-dropdown">
        <a className="navbar__link" href="/login" aria-label="Log in">
          Log in
        </a>
      </div>
    );
  }
};

export default UserMenu;