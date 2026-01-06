// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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

module.exports = sidebars;