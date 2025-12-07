import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Robotics Curriculum',
  tagline: 'Learn robotics from fundamentals to advanced humanoid systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-curriculum.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'msaad-ai', // Your GitHub org/user name.
  projectName: 'physical-ai-curriculum', // Your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Use the tutorialSidebar as the default sidebar
          sidebarCollapsed: false,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/msaad-ai/physical-ai-curriculum/edit/main/',
        },
        blog: false, // Disable blog functionality
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
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {to: '/', label: 'Home', position: 'left'},
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            {
              label: 'Module 1: The Robotic Nervous System (ROS 2)',
              to: '/docs/module1/intro',
            },
            {
              label: 'Module 2: Digital Twin (Gazebo & Unity)',
              to: '/docs/module2/intro',
            },
            {
              label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
              to: '/docs/module3/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action (VLA)',
              to: '/docs/module4',
            },
          ],
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: The Robotic Nervous System (ROS 2)',
              to: '/docs/module1/intro',
            },
            {
              label: 'Module 2: Digital Twin (Gazebo & Unity)',
              to: '/docs/module2/intro',
            },
            {
              label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
              to: '/docs/module3/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action (VLA)',
              to: '/docs/module4',
            },
          ],
        },
        {
          title: 'Quick Links',
          items: [
            {
              label: 'Home',
              to: '/',
            },
            {
              label: 'Documentation Overview',
              to: '/docs/intro',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/msaad-ai/physical-ai-curriculum',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Support / Contact',
              href: '#',
            },
            {
              label: 'Course Certificate',
              href: '#',
            },
            {
              label: 'Community Forum',
              href: '#',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
