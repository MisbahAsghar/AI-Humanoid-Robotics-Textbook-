// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Physical AI & Humanoid Robotics Textbook
  textbookSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations',
      collapsed: false,
      items: [
        'textbook/ch01',
        'textbook/ch02',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Core Technologies',
      collapsed: false,
      items: [
        'textbook/ch03',
        'textbook/ch04',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Simulation Environments',
      collapsed: false,
      items: [
        'textbook/ch05',
        'textbook/ch06',
        'textbook/ch07',
        'textbook/ch08',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Advanced AI Systems',
      collapsed: false,
      items: [
        'textbook/ch09',
        'textbook/ch10',
      ],
    },
    {
      type: 'category',
      label: 'Part 5: Integration & Deployment',
      collapsed: false,
      items: [
        'textbook/ch11',
        'textbook/ch12',
      ],
    },
  ],
};

export default sidebars;
