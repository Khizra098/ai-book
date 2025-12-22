import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'index',
    'introduction',
    {
      type: 'category',
      label: 'Foundations',
      items: [
        'foundations/ai-first-architecture',
        'foundations/data-centric-development',
        'foundations/prompt-engineering'
      ],
    },
    {
      type: 'category',
      label: 'Implementation',
      items: [
        'implementation/ai-assisted-generation',
        'implementation/ai-enhanced-testing',
        'implementation/ai-powered-debugging',
        'implementation/model-integration'
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'advanced/real-time-ai',
        'advanced/ai-security',
        'advanced/performance-optimization',
        'advanced/monitoring-observability'
      ],
    },
    {
      type: 'category',
      label: 'Future Directions',
      items: [
        'future/emerging-tools',
        'future/ethical-ai',
        'future/conclusion'
      ],
    },
  ],
};

export default sidebars;
