import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physically AI & Humanoid Robots Textbook',
  tagline: 'Building AI-Integrated Applications',
  favicon: 'img/ai%20robort.png',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://ai-book-kohl-seven.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Khizra', // Usually your GitHub org/user name.
  projectName: 'ai-native-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  // i18n: {
  //   defaultLocale: 'en',
  //   locales: ['en', 'es', 'fr', 'ur'], // Added Spanish, French, and Urdu
  
  //   path: '../i18n',
  // },
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',
    },
  },
},

  plugins: [
  ],
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/ai-native-book/ai-native-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'AI Native Book',
      logo: {
        alt: 'AI Native Book Logo',
        src: 'img/ai robort.png', // actual filename contains a space in the repo
        // Set explicit size so the PNG displays crisply on desktop & mobile
        width: 36,
        height: 36,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Docs',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          to: '/login',
          label: 'Log in',
          position: 'right',
        },
        {
          href: 'https://github.com/Khizra098',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'LinkedIn',
              to: 'https://www.linkedin.com/in/khizra-younus-1757742b6?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Facebook',
              href: 'https://www.facebook.com/share/17yqnGc9Bd/',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/fSksqKnC',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Khizra098',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI Native Book. Built with Khizra❤️`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
