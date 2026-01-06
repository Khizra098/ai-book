import React, { useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import '../../i18n/ur/code.json';


function ClientRoot({ children }) {
  useEffect(() => {
    if (!ExecutionEnvironment.canUseDOM) {
      return;
    }

    // Initialize user menu functionality
    let userMenuInitialized = false;
    const initializeUserMenu = () => {
      if (userMenuInitialized) return;
      userMenuInitialized = true;

      const createAndInjectUserMenu = () => {
        try {
          // Check if user is logged in
          const isLoggedIn = localStorage.getItem('authToken') !== null;

          // Find the login link(s) in navbar to replace with user menu
          const loginLinks = Array.from(document.querySelectorAll('a[href]')).filter((a) => {
            try {
              const url = new URL(a.href, window.location.href);
              const pathname = url.pathname.replace(/\/+$/,'');
              return pathname.endsWith('/login') || pathname === '/login';
            } catch (e) {
              return false;
            }
          });
          if (loginLinks.length > 0) {
            // Get the login link that's in the navbar (not in content)
            let loginLink = null;
            for (const link of loginLinks) {
              // Check if the link is in navbar by looking at parent classes
              let parent = link.parentElement;
              let inNavbar = false;
              while (parent && parent !== document.body) {
                if (parent.classList &&
                    (parent.classList.contains('navbar__item') ||
                     parent.classList.contains('navbar'))) {
                  inNavbar = true;
                  break;
                }
                parent = parent.parentElement;
              }

              if (inNavbar) {
                loginLink = link;
                break;
              }
            }

            if (loginLink) {
              // Remove existing user menu if present
              const existingUserMenu = document.querySelector('#user-menu-container');
              if (existingUserMenu) {
                existingUserMenu.remove();
              }

              if (isLoggedIn) {
                // Create user menu dropdown
                const userMenuContainer = document.createElement('div');
                userMenuContainer.id = 'user-menu-container';
                userMenuContainer.className = 'navbar__item dropdown dropdown--right';

                userMenuContainer.innerHTML = `
                  <a class="navbar__link" href="#" id="user-menu-trigger" aria-haspopup="true" aria-expanded="false">
                    <span class="navbar__user-icon">👤</span>
                  </a>
                  <ul class="dropdown__menu" id="user-menu-dropdown" style="display: none;">
                    <li><a class="dropdown__link" href="/profile">Profile</a></li>
                    <li><a class="dropdown__link" href="/settings">Settings</a></li>
                    <li><a class="dropdown__link" href="#" id="logout-link">Log Out</a></li>
                  </ul>
                `;

                // Replace the login link with user menu
                if (loginLink.parentNode) {
                  loginLink.parentNode.replaceChild(userMenuContainer, loginLink);

                  // Add event listeners
                  const trigger = document.getElementById('user-menu-trigger');
                  const dropdown = document.getElementById('user-menu-dropdown');
                  const logoutLink = document.getElementById('logout-link');

                  if (trigger && dropdown) {
                    // Toggle dropdown on click
                    const toggleDropdown = (e) => {
                      e.preventDefault();
                      const isExpanded = trigger.getAttribute('aria-expanded') === 'true';
                      trigger.setAttribute('aria-expanded', String(!isExpanded));
                      dropdown.style.display = isExpanded ? 'none' : 'block';
                    };

                    trigger.addEventListener('click', toggleDropdown);

                    // Close dropdown when clicking outside
                    const closeDropdown = (e) => {
                      if (!userMenuContainer.contains(e.target)) {
                        trigger.setAttribute('aria-expanded', 'false');
                        dropdown.style.display = 'none';
                      }
                    };

                    document.addEventListener('click', closeDropdown);

                    // Prevent dropdown from closing when clicking inside the dropdown
                    dropdown.addEventListener('click', (e) => {
                      e.stopPropagation();
                    });

                    // Add click handlers to profile and settings links to close dropdown before navigation
                    const profileLink = dropdown.querySelector('a[href="/profile"]');
                    const settingsLink = dropdown.querySelector('a[href="/settings"]');

                    if (profileLink) {
                      profileLink.addEventListener('click', () => {
                        // Close the dropdown immediately
                        trigger.setAttribute('aria-expanded', 'false');
                        dropdown.style.display = 'none';
                      });
                    }

                    if (settingsLink) {
                      settingsLink.addEventListener('click', () => {
                        // Close the dropdown immediately
                        trigger.setAttribute('aria-expanded', 'false');
                        dropdown.style.display = 'none';
                      });
                    }

                    // Store cleanup functions to remove later
                    userMenuContainer._cleanup = () => {
                      trigger.removeEventListener('click', toggleDropdown);
                      document.removeEventListener('click', closeDropdown);
                    };
                  }

                  if (logoutLink) {
                    const handleLogout = (e) => {
                      e.preventDefault();

                      // Get the old token value for the storage event
                      const oldToken = localStorage.getItem('authToken');

                      localStorage.removeItem('authToken');

                      // Dispatch a storage event to trigger UI updates in other parts of the app
                      window.dispatchEvent(new StorageEvent('storage', {
                        key: 'authToken',
                        newValue: null,
                        oldValue: oldToken,
                        url: window.location.href
                      }));

                      window.location.href = '/';
                    };
                    logoutLink.addEventListener('click', handleLogout);

                    // Store cleanup function
                    userMenuContainer._logoutCleanup = () => {
                      logoutLink.removeEventListener('click', handleLogout);
                    };
                  }
                }
              } else {
                // If not logged in, ensure login link is visible (in case it was replaced)
                // The login link is already in the navbar via config, so no action needed
              }
            }
          }
        } catch (error) {
          console.error('Error in createAndInjectUserMenu:', error);
        }
      };

      // Run immediately
      createAndInjectUserMenu();

      // Listen for storage events to update UI when login status changes in other tabs
      const handleStorageChange = (e) => {
        if (e.key === 'authToken') {
          // Add a small delay to ensure the storage has been fully updated
          setTimeout(() => {
            // Remove existing user menu first to clean up event listeners
            const existingUserMenu = document.querySelector('#user-menu-container');
            if (existingUserMenu && existingUserMenu._cleanup) {
              existingUserMenu._cleanup();
            }
            if (existingUserMenu && existingUserMenu._logoutCleanup) {
              existingUserMenu._logoutCleanup();
            }

            // Re-create the menu with new state
            createAndInjectUserMenu();
          }, 10);
        }
      };

      window.addEventListener('storage', handleStorageChange);

      // Store cleanup function
      window._userMenuCleanup = () => {
        window.removeEventListener('storage', handleStorageChange);

        // Clean up any existing user menu
        const existingUserMenu = document.querySelector('#user-menu-container');
        if (existingUserMenu && existingUserMenu._cleanup) {
          existingUserMenu._cleanup();
        }
        if (existingUserMenu && existingUserMenu._logoutCleanup) {
          existingUserMenu._logoutCleanup();
        }
      };
    };


    // Fix locale dropdown: ensure triggers have href, menu is interactable, and provide a safe click fallback
    const fixLocaleDropdown = () => {
      try {
        const triggers = Array.from(document.querySelectorAll('.navbar__item.dropdown .navbar__link'));
        triggers.forEach((t) => {
          if (!t) return;
          // Add href if missing so CSS doesn't disable pointer-events
          let addedHref = false;
          if (!t.hasAttribute('href')) {
            t.setAttribute('href', '#');
            addedHref = true;
          }
          // Expose role for accessibility
          if (!t.hasAttribute('role')) t.setAttribute('role', 'button');

          const parent = t.closest('.dropdown');
          const menu = parent ? parent.querySelector('.dropdown__menu') : null;
          if (menu) {
            // Ensure the menu can appear above other content
            menu.style.zIndex = menu.style.zIndex || '2000';
            // Ensure the menu is interactive by default
            if (!menu.style.pointerEvents) menu.style.pointerEvents = 'auto';
          }

          // Add a safe click fallback to toggle dropdown when Docusaurus handler doesn't run
          // Only apply this to non-locale dropdowns to avoid interfering with language switching
          // Check if this is the locale dropdown by looking for specific characteristics
          const isLocaleDropdown = t.closest('.navbar__item.dropdown') &&
                                  (t.closest('.navbar__item.dropdown')?.querySelector('.dropdown__menu a[href*="/locale/"]') ||
                                   t.closest('.navbar__item.dropdown')?.classList.contains('dropdown--languages') ||
                                   t.closest('.navbar__item.dropdown')?.querySelector('[href*="locale"]'));

          if (!t._localeFallbackHandler && !isLocaleDropdown) {
            const fallback = (e) => {
              e.preventDefault();
              if (!parent) return;
              const isShown = parent.classList.toggle('dropdown--show');
              // Update aria attribute for accessibility
              t.setAttribute('aria-expanded', String(isShown));
            };
            t.addEventListener('click', fallback);
            t._localeFallbackHandler = fallback;
            parent && (parent._localeCleanup = () => {
              t.removeEventListener('click', fallback);
              delete t._localeFallbackHandler;
            });

            // Store what we added so cleanup can revert it
            t._localeAddedHref = addedHref;
            t._localeAddedRole = !t.hasAttribute('role') ? false : true;
          }
        });

        window._localeDropdownFixCleanup = () => {
          triggers.forEach((t) => {
            if (!t) return;
            const parent = t.closest('.dropdown');
            const menu = parent ? parent.querySelector('.dropdown__menu') : null;
            if (parent && parent._localeCleanup) {
              parent._localeCleanup();
              delete parent._localeCleanup;
            }
            if (menu) {
              menu.style.zIndex = '';
              menu.style.pointerEvents = '';
            }
            if (t._localeAddedHref) {
              if (t.getAttribute('href') === '#') t.removeAttribute('href');
              delete t._localeAddedHref;
            }
            if (t._localeFallbackHandler) {
              t.removeEventListener('click', t._localeFallbackHandler);
              delete t._localeFallbackHandler;
            }
            if (t._localeAddedRole === false) {
              t.removeAttribute('role');
              delete t._localeAddedRole;
            }
          });
          delete window._localeDropdownFixCleanup;
        };
      } catch (err) {
        console.error('Error fixing locale dropdown:', err);
      }
    };

    const initializeChatbot = async () => {
      try {
        // Wait for DOM to be ready
        if (document.readyState === 'loading') {
          await new Promise(resolve => {
            const onReady = () => {
              resolve();
              document.removeEventListener('DOMContentLoaded', onReady);
            };
            document.addEventListener('DOMContentLoaded', onReady);
          });
        }


        // Create or find the chatbot container
        let container = document.getElementById('rag-chatbot-container');
        if (!container) {
          container = document.createElement('div');
          container.id = 'rag-chatbot-container';
          container.style.position = 'fixed';
          container.style.bottom = '20px';
          container.style.right = '20px';
          container.style.zIndex = '1000';
          container.style.width = '400px';
          container.style.height = '500px';
          document.body.appendChild(container);
        }

        // Add a small delay to ensure DOM is ready
        await new Promise(resolve => setTimeout(resolve, 100));

        // Dynamically import and render the RAG Chatbot
        const [{ default: RAGChatbot }, { createRoot }] = await Promise.all([
          import('../components/RAGChatbot'),
          import('react-dom/client')
        ]);

        const ChatbotWrapper = () => (
          <RAGChatbot
            bookId={window.location.pathname.replace(/\//g, '_') || 'default_book'}
            apiEndpoint={window.RAG_CHATBOT_API_ENDPOINT || 'http://localhost:8000'}
          />
        );

        // Clear any existing content and render the new component
        container.innerHTML = '';
        const root = createRoot(container);
        root.render(<ChatbotWrapper />);

        console.log('RAG Chatbot initialized successfully');
      } catch (error) {
        console.error('Failed to initialize RAG Chatbot:', error);
        console.error('Error stack:', error.stack);
      }
    };

    // Initialize both functionalities
    initializeUserMenu();
    initializeChatbot();
    // Run locale dropdown fix after initialization (small delay to ensure navbar rendered)
    setTimeout(fixLocaleDropdown, 50);

    // Cleanup function
    return () => {
      if (window._userMenuCleanup) {
        window._userMenuCleanup();
        window._userMenuCleanup = null;
      }
      if (window._localeDropdownFixCleanup) {
        window._localeDropdownFixCleanup();
        window._localeDropdownFixCleanup = null;
      }
    };
  }, []);

  return <>{children}</>;
}

export default ClientRoot;



// import React, { useEffect } from 'react';
// import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// function ClientRoot({ children }) {
//   useEffect(() => {
//     if (!ExecutionEnvironment.canUseDOM) return;

//     /* =========================
//        USER MENU (unchanged)
//     ========================== */
//     let userMenuInitialized = false;

//     const initializeUserMenu = () => {
//       if (userMenuInitialized) return;
//       userMenuInitialized = true;

//       const createAndInjectUserMenu = () => {
//         try {
//           const isLoggedIn = localStorage.getItem('authToken') !== null;
//           const loginLinks = document.querySelectorAll(
//             'a[href="/login"], a[href$="/login"]'
//           );

//           let loginLink = null;
//           for (const link of loginLinks) {
//             let parent = link.parentElement;
//             while (parent && parent !== document.body) {
//               if (
//                 parent.classList?.contains('navbar__item') ||
//                 parent.classList?.contains('navbar')
//               ) {
//                 loginLink = link;
//                 break;
//               }
//               parent = parent.parentElement;
//             }
//             if (loginLink) break;
//           }

//           if (!loginLink) return;

//           const existing = document.getElementById('user-menu-container');
//           if (existing) existing.remove();

//           if (!isLoggedIn) return;

//           const container = document.createElement('div');
//           container.id = 'user-menu-container';
//           container.className =
//             'navbar__item dropdown dropdown--hoverable dropdown--right';

//           container.innerHTML = `
//             <a class="navbar__link" href="#" id="user-menu-trigger">
//               👤
//             </a>
//             <ul class="dropdown__menu">
//               <li><a class="dropdown__link" href="/profile">Profile</a></li>
//               <li><a class="dropdown__link" href="/settings">Settings</a></li>
//               <li><a class="dropdown__link" href="#" id="logout-link">Log Out</a></li>
//             </ul>
//           `;

//           loginLink.parentNode.replaceChild(container, loginLink);

//           const trigger = container.querySelector('#user-menu-trigger');
//           const menu = container.querySelector('.dropdown__menu');
//           const logout = container.querySelector('#logout-link');

//           trigger.addEventListener('click', (e) => {
//             e.preventDefault();
//             container.classList.toggle('dropdown--show');
//             menu.style.display = container.classList.contains('dropdown--show')
//               ? 'block'
//               : 'none';
//           });

//           logout.addEventListener('click', (e) => {
//             e.preventDefault();
//             localStorage.removeItem('authToken');
//             window.location.href = '/';
//           });
//         } catch (err) {
//           console.error(err);
//         }
//       };

//       createAndInjectUserMenu();
//     };

//     /* =========================
//        ✅ FIXED LOCALE DROPDOWN
//     ========================== */
//     const fixLocaleDropdown = () => {
//       try {
//         const triggers = document.querySelectorAll(
//           '.navbar__item.dropdown .navbar__link'
//         );

//         triggers.forEach((t) => {
//           if (t._localeFixed) return;
//           t._localeFixed = true;

//           if (!t.hasAttribute('role')) {
//             t.setAttribute('role', 'button');
//           }

//           t.addEventListener('click', (e) => {
//             const href = t.getAttribute('href');

//             // ✅ Allow real navigation (language switch)
//             if (href && href !== '#') {
//               return;
//             }

//             e.preventDefault();

//             const parent = t.closest('.dropdown');
//             if (!parent) return;

//             parent.classList.toggle('dropdown--show');
//           });
//         });
//       } catch (err) {
//         console.error('Locale dropdown error:', err);
//       }
//     };

//     /* =========================
//        CHATBOT (unchanged)
//     ========================== */
//     const initializeChatbot = async () => {
//       try {
//         if (document.readyState === 'loading') {
//           await new Promise((r) =>
//             document.addEventListener('DOMContentLoaded', r, { once: true })
//           );
//         }

//         fixLocaleDropdown();

//         let container = document.getElementById('rag-chatbot-container');
//         if (!container) {
//           container = document.createElement('div');
//           container.id = 'rag-chatbot-container';
//           Object.assign(container.style, {
//             position: 'fixed',
//             bottom: '20px',
//             right: '20px',
//             width: '400px',
//             height: '500px',
//             zIndex: 1000,
//           });
//           document.body.appendChild(container);
//         }

//         const [{ default: RAGChatbot }, { createRoot }] = await Promise.all([
//           import('../components/RAGChatbot'),
//           import('react-dom/client'),
//         ]);

//         const root = createRoot(container);
//         root.render(
//           <RAGChatbot
//             bookId={window.location.pathname.replace(/\//g, '_') || 'default'}
//             apiEndpoint={
//               window.RAG_CHATBOT_API_ENDPOINT || 'http://localhost:8000'
//             }
//           />
//         );
//       } catch (err) {
//         console.error(err);
//       }
//     };

//     initializeUserMenu();
//     initializeChatbot();
//   }, []);

//   return <>{children}</>;
// }

// export default ClientRoot;


