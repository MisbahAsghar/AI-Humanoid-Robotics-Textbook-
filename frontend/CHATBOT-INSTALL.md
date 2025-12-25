# Chatbot Installation Instructions

Add this to `docusaurus.config.js`:

```javascript
// Add to themeConfig section, after prism:
scripts: [
  {
    src: '/chatbot.js',
    async: true,
  },
],
```

Full location in config:

```javascript
themeConfig: {
  // ... existing config ...
  prism: {
    theme: prismThemes.github,
    darkTheme: prismThemes.dracula,
  },
  scripts: [
    {
      src: '/chatbot.js',
      async: true,
    },
  ],
}
```

Chatbot will appear on all pages as a floating button in bottom-right corner.
