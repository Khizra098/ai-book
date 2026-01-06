import type {ReactNode} from 'react';
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

// Lazy load components for better performance
const HeroSection = React.lazy(() => import('../components/Homepage/HeroSection/HeroSection'));
const ChapterDropdown = React.lazy(() => import('../components/Homepage/ChapterNavigation/ChapterDropdown'));
const ModuleCards = React.lazy(() => import('../components/Homepage/ModuleCards/ModuleCards'));

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <HeroSection />
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={` ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book">
      <React.Suspense fallback={<div>Loading...</div>}>
        <HomepageHeader />
        <ChapterDropdown />
        <ModuleCards />
      </React.Suspense>
    </Layout>
  );
}
