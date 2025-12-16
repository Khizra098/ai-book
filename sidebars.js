// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 for Humanoid Robotics',
      items: [
        'ros2-nervous-system/introduction-to-ros2',
        'ros2-nervous-system/ros2-communication',
        'ros2-nervous-system/urdf-robot-structure',
      ],
    },
    // Add more categories as needed
  ],
};

module.exports = sidebars;