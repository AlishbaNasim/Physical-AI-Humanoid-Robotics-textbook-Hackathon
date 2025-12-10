# Quickstart Guide: Docusaurus Book for Physical AI & Humanoid Robotics

## Overview
This guide provides a quick setup and development workflow for the Physical AI & Humanoid Robotics book built with Docusaurus.

## Prerequisites
- Node.js 18 or higher
- npm or yarn package manager
- Git
- Basic knowledge of Markdown and JavaScript/React (for custom components)

## Setup Instructions

### 1. Clone and Initialize
```bash
# If starting from scratch
npx create-docusaurus@latest my-website classic
cd my-website

# If working with existing repository
git clone <repository-url>
cd my-website
npm install
```

### 2. Install Additional Dependencies
```bash
# For LaTeX support (needed for mathematical equations in robotics content)
npm install @docusaurus/module-type-aliases
npm install mathjax

# For advanced search (optional)
npm install @docusaurus/theme-search-algolia
```

### 3. Project Structure Setup
Create the directory structure as planned:

```bash
# Content directories
mkdir -p docs/{chapter-1-introduction,chapter-2-ros2-basics,chapter-3-python-agents-rclpy,chapter-4-urdf-humanoids,chapter-5-practical-system,chapter-6-summary}

# Examples directory
mkdir -p examples/{python-agents,urdf-models,ros2-workflows}

# Static assets
mkdir -p static/{img/{chapter-1,chapter-2,chapter-6},media}
```

### 4. Add Initial Content
```bash
# Create initial chapter files
touch docs/chapter-1-introduction/{01-overview.md,02-physical-ai.md,03-role-of-ros2.md}
touch docs/chapter-2-ros2-basics/{01-nodes.md,02-topics.md,03-services.md,04-actions.md,05-code-examples.md}
touch docs/chapter-3-python-agents-rclpy/{01-intro-to-rclpy.md,02-python-agent-setup.md,03-joint-control-examples.md,04-sensor-integration.md}
touch docs/chapter-4-urdf-humanoids/{01-urdf-basics.md,02-links-joints.md,03-sensors.md,04-gazebo-integration.md}
touch docs/chapter-5-practical-system/{01-building-control-pipeline.md,02-ros2-composition.md,03-python-command-to-movement.md}
touch docs/chapter-6-summary/{01-key-takeaways.md,02-exercises.md,03-references.md}
```

### 5. Configure Docusaurus
Update `docusaurus.config.js` with your site configuration:

```javascript
// docusaurus.config.js
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to ROS 2 and humanoid robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/my-website/',

  // GitHub pages deployment config.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'my-website', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

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
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/your-username/my-website',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Content',
            items: [
              {
                label: 'Introduction',
                to: '/docs/chapter-1-introduction/01-overview',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/my-website',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
```

### 6. Configure Sidebars
Update `sidebars.js` to reflect the chapter structure:

```javascript
// sidebars.js
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: Introduction',
      items: [
        'chapter-1-introduction/01-overview',
        'chapter-1-introduction/02-physical-ai',
        'chapter-1-introduction/03-role-of-ros2'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 Basics',
      items: [
        'chapter-2-ros2-basics/01-nodes',
        'chapter-2-ros2-basics/02-topics',
        'chapter-2-ros2-basics/03-services',
        'chapter-2-ros2-basics/04-actions',
        'chapter-2-ros2-basics/05-code-examples'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Python Agents with rclpy',
      items: [
        'chapter-3-python-agents-rclpy/01-intro-to-rclpy',
        'chapter-3-python-agents-rclpy/02-python-agent-setup',
        'chapter-3-python-agents-rclpy/03-joint-control-examples',
        'chapter-3-python-agents-rclpy/04-sensor-integration'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: URDF for Humanoids',
      items: [
        'chapter-4-urdf-humanoids/01-urdf-basics',
        'chapter-4-urdf-humanoids/02-links-joints',
        'chapter-4-urdf-humanoids/03-sensors',
        'chapter-4-urdf-humanoids/04-gazebo-integration'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Practical System',
      items: [
        'chapter-5-practical-system/01-building-control-pipeline',
        'chapter-5-practical-system/02-ros2-composition',
        'chapter-5-practical-system/03-python-command-to-movement'
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6: Summary',
      items: [
        'chapter-6-summary/01-key-takeaways',
        'chapter-6-summary/02-exercises',
        'chapter-6-summary/03-references'
      ],
    },
  ],
};

module.exports = sidebars;
```

## Development Workflow

### Start Development Server
```bash
npm run start
# Runs the development server at http://localhost:3000
```

### Build for Production
```bash
npm run build
# Creates a static build in the build/ directory
```

### Deploy to GitHub Pages
```bash
npm run deploy
# Deploys the site to GitHub Pages
```

## Content Creation Guidelines

### Adding New Chapters/Sections
1. Create the markdown file in the appropriate chapter directory
2. Add the file path to the `sidebars.js` configuration
3. Follow the naming convention: `NN-title.md` (e.g., `01-introduction.md`)

### Adding Code Examples
1. Place code files in the appropriate subdirectory of `examples/`
2. Reference them in markdown using Docusaurus code block syntax:
   ```markdown
   import {CodeBlock} from './path/to/code.py';
   ```

### Adding Images
1. Place images in the appropriate subdirectory of `static/img/`
2. Reference them in markdown:
   ```markdown
   ![Alt text](/img/chapter-1/image-name.png)
   ```

### Adding Citations
Use standard markdown for APA-style citations:
```markdown
According to Smith et al. (2023), this is an important concept [^1].

[^1]: Smith, J., Doe, A., & Brown, C. (2023). Important robotics concepts. *Journal of Robotics*, 45(2), 123-145.
```

## Verification Steps
1. Run `npm run build` to ensure the site builds without errors
2. Check all links work correctly
3. Verify all code examples are properly formatted
4. Confirm all images display correctly
5. Test navigation and search functionality