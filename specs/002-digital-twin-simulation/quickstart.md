# Quickstart Guide: Physical AI & Humanoid Robotics - The Digital Twin (Gazebo & Unity)

**Date**: 2026-01-13
**Feature**: Physical AI & Humanoid Robotics - The Digital Twin (Gazebo & Unity)

## Overview

This quickstart guide provides the essential steps to set up and begin working with the digital twin simulation educational module. It covers the installation, configuration, and initial content creation for the Docusaurus-based educational platform focused on Gazebo and Unity simulation.

## Prerequisites

Before starting, ensure you have the following:

- **Node.js**: Version 18 or higher
- **npm or yarn**: Package manager for Node.js
- **Git**: Version control system
- **Basic knowledge**: Familiarity with command-line interfaces and basic 3D graphics/simulation concepts
- **IDE/Editor**: Any modern code editor (VS Code recommended)

## Step 1: Environment Setup

### Install Node.js
```bash
# Using Node Version Manager (recommended)
nvm install 18
nvm use 18

# Or download directly from nodejs.org
```

### Verify installation
```bash
node --version  # Should show v18.x.x or higher
npm --version   # Should show version number
```

## Step 2: Project Initialization

### Navigate to the website directory
```bash
cd website
```

### Install additional dependencies if needed
```bash
# Navigate to website directory
npm install @docusaurus/module-type-aliases @docusaurus/types
```

## Step 3: Project Structure Setup

The module structure is already set up in the docs directory:

```bash
# Module directory structure
docs/modules/digital-twin-simulation/
├── index.md
├── chapter-1-gazebo-physics-simulation.md
├── chapter-2-unity-interaction-visualization.md
└── chapter-3-digital-twin-concepts.md
```

## Step 4: Configuration

### Update docusaurus.config.js
Ensure the module is added to your sidebar and navigation:

```javascript
// website/docusaurus.config.js
module.exports = {
  // ... other config
  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/your-repo/edit/main/',
        },
        // ... other preset options
      }),
    ],
  ],
  // ... rest of config
};
```

### Update sidebars.js
```javascript
// website/sidebars.js
module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'modules/digital-twin-simulation/index',
        'modules/digital-twin-simulation/chapter-1-gazebo-physics-simulation',
        'modules/digital-twin-simulation/chapter-2-unity-interaction-visualization',
        'modules/digital-twin-simulation/chapter-3-digital-twin-concepts'
      ],
    },
    // ... other items
  ],
};
```

## Step 5: Content Creation

### Create the module index
```bash
# Create the index file for the module
touch docs/modules/digital-twin-simulation/index.md
```

### Content structure for each chapter

Each chapter should follow this structure:

```markdown
---
sidebar_position: [position]
title: '[Chapter Title]'
description: '[Brief description]'
---

# [Chapter Title]

## Introduction

[Provide an overview of the chapter content]

## Main Concepts

[Detail the core concepts covered in this chapter]

## Practical Examples

[Include practical examples and code snippets where applicable]

## Summary

[Recap the key points covered in the chapter]

## Learning Objectives Review

[Summarize what the learner should understand after completing the chapter]
```

## Step 6: Development Server

Start the development server to see your changes in real-time:

```bash
# From the website directory
npm run start
```

This will start a local server at http://localhost:3000 where you can view and test your documentation.

## Step 7: Build for Production

To build the static site for deployment:

```bash
# From the website directory
npm run build
```

The built site will be available in the `build/` directory.

## Step 8: Deployment Preparation

### GitHub Pages Setup

1. Ensure your repository is connected to GitHub
2. In your GitHub repository settings, go to "Pages"
3. Select source as "GitHub Actions"
4. The deployment will be handled automatically

### Deployment Script

The deployment is configured in package.json:

```bash
# Deploy to GitHub Pages (configured in package.json)
npm run deploy
```

## Next Steps

1. **Content Creation**: Begin writing the actual chapter content following the specifications
2. **Review Process**: Establish review cycles for technical accuracy of simulation concepts
3. **Testing**: Validate all examples and exercises work as expected
4. **Personalization**: Prepare for future personalization features
5. **AI Integration**: Structure content for future AI-based enhancements

## Simulation-Specific Considerations

### Gazebo Concepts
- Focus on physics simulation fundamentals
- Emphasize realistic environment setup
- Cover sensor simulation thoroughly

### Unity Concepts
- Highlight visualization capabilities
- Explain human-robot interaction concepts
- Demonstrate data linking between systems

### Digital Twin Concepts
- Explain synchronization between real and virtual
- Cover planning and testing methodologies
- Connect simulation to real-world applications

## Troubleshooting

### Common Issues

- **Page Not Found**: Verify sidebar configuration matches file paths
- **Build Errors**: Check for syntax errors in Markdown files
- **Navigation Issues**: Ensure all navigation links are correctly formatted
- **Content Rendering**: Verify frontmatter is properly formatted