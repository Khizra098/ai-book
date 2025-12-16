import React from 'react';
import { useLocation } from '@docusaurus/router';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';
import { PageMetadata, SkipToContentFallbackId } from '@docusaurus/theme-common';
import { useKeyboardNavigation } from '@docusaurus/theme-common/internal';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './LayoutWrapper.module.css';

function LayoutWrapper(props) {
  const { children, ...layoutProps } = props;
  const { sidebarName, docsSidebars } = useDocsSidebar();
  const location = useLocation();
  const keyboardNavigation = useKeyboardNavigation();

  // Determine if we should show sidebar based on current page
  const showSidebar = sidebarName && docsSidebars[sidebarName] &&
                     !location.pathname.includes('/pages/') &&
                     !location.pathname.includes('/blog/');

  return (
    <Layout {...layoutProps}>
      <div className={clsx('container padding-vert--lg', styles.layoutWrapper)}>
        <div className="row">
          {showSidebar && (
            <div className="col col--3">
              <div className={styles.sidebarContainer}>
                <div className={styles.stickySidebar} id="sticky-sidebar">
                  {/* The actual sidebar content will be rendered by Docusaurus */}
                  {/* We're just providing a container with sticky positioning */}
                </div>
              </div>
            </div>
          )}
          <div className={showSidebar ? "col col--7" : "col col--9"}>
            <div className={styles.mainContent}>
              <main id={SkipToContentFallbackId}>
                {children}
              </main>
            </div>
          </div>
          <div className="col col--2">
            {/* Additional right sidebar or metadata area */}
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default LayoutWrapper;