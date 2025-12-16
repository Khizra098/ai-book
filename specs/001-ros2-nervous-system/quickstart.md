# Quickstart: ROS 2 Educational Module Development

## Prerequisites

1. **Node.js**: Version 18 or higher (LTS recommended)
2. **npm or yarn**: Package manager (comes with Node.js)
3. **Git**: Version control system
4. **Text Editor**: VS Code or similar with Markdown support

## Setup Docusaurus Environment

1. **Install Docusaurus globally** (if not already installed):
   ```bash
   npm install -g @docusaurus/init
   ```

2. **Navigate to project directory**:
   ```bash
   cd /path/to/ai-native-book
   ```

3. **Install project dependencies**:
   ```bash
   npm install
   ```

4. **Start local development server**:
   ```bash
   npm run start
   ```
   This will start a local server at `http://localhost:3000` with live reloading.

## Add the ROS 2 Module Content

1. **Create module directory**:
   ```bash
   mkdir -p docs/ros2-nervous-system
   ```

2. **Create the three chapter files**:
   - `docs/ros2-nervous-system/introduction-to-ros2.md`
   - `docs/ros2-nervous-system/ros2-communication.md`
   - `docs/ros2-nervous-system/urdf-robot-structure.md`

3. **Add content to each file** following Docusaurus Markdown format.

## Configure Navigation

1. **Update sidebar configuration** in `sidebars.js`:
   ```javascript
   module.exports = {
     // ... existing sidebar configuration
     ros2NervousSystem: [
       'ros2-nervous-system/introduction-to-ros2',
       'ros2-nervous-system/ros2-communication',
       'ros2-nervous-system/urdf-robot-structure',
     ],
   };
   ```

2. **Verify the module appears in navigation** when running the development server.

## Build and Deploy

1. **Build static files**:
   ```bash
   npm run build
   ```

2. **Test locally**:
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages** following your project's deployment process.

## Content Creation Guidelines

- Use Docusaurus Markdown features for better documentation
- Include code examples with proper syntax highlighting
- Add references to official ROS 2 documentation
- Ensure all code examples are verified and runnable
- Use appropriate headings structure (H1 for page title, H2-H4 for sections)