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
  // Combined sidebar showing all modules in one place
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI Humanoid Robotics Book',
      items: [
        'index', // Main index page
        {
          type: 'category',
          label: 'Physical AI & Humanoid Robotics Book',
          items: [
            'physical-ai-humanoid-book/introduction-to-physical-ai',
            'physical-ai-humanoid-book/embodied-intelligence-overview',
            'physical-ai-humanoid-book/hardware-lab-setup',
            'physical-ai-humanoid-book/weekly-learning-plan',
            'physical-ai-humanoid-book/assessments',
            'physical-ai-humanoid-book/capstone-overview',
            'physical-ai-humanoid-book/references',
          ],
        },
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'module-1-ros2-nervous-system/chapter-1-introduction-to-ros2-middleware',
            'module-1-ros2-nervous-system/chapter-2-ros2-nodes-topics-services',
            'module-1-ros2-nervous-system/chapter-3-bridging-python-agents-to-ros-controllers',
            'module-1-ros2-nervous-system/chapter-4-understanding-urdf-for-humanoids',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'module-2-digital-twin-sim/chapter-1-physics-simulation-environment-building',
            'module-2-digital-twin-sim/chapter-2-simulating-physics-gravity-collisions',
            'module-2-digital-twin-sim/chapter-3-high-fidelity-rendering-hri',
            'module-2-digital-twin-sim/chapter-4-simulating-sensors-lidar-depth-imu',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'module-3-isaac-integration/chapter-1-advanced-perception-training',
            'module-3-isaac-integration/chapter-2-isaac-sim-simulation-data',
            'module-3-isaac-integration/chapter-3-isaac-ros-vslam-navigation',
            'module-3-isaac-integration/chapter-4-nav2-bipedal-path-planning',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            'module-4-vla-integration/chapter-1-llms-robotics-convergence',
            'module-4-vla-integration/chapter-2-voice-to-action-whisper',
            'module-4-vla-integration/chapter-3-cognitive-planning-llms',
            'module-4-vla-integration/chapter-4-capstone-autonomous-humanoid',
          ],
        },
      ],
    },
  ],

  // Keep individual sidebars for specific module navigation if needed
  ros2NervousSystemSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2-nervous-system/chapter-1-introduction-to-ros2-middleware',
        'module-1-ros2-nervous-system/chapter-2-ros2-nodes-topics-services',
        'module-1-ros2-nervous-system/chapter-3-bridging-python-agents-to-ros-controllers',
        'module-1-ros2-nervous-system/chapter-4-understanding-urdf-for-humanoids',
      ],
    },
  ],

  // Manual sidebar for Digital Twin Simulation module
  digitalTwinSimSidebar: [
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin-sim/chapter-1-physics-simulation-environment-building',
        'module-2-digital-twin-sim/chapter-2-simulating-physics-gravity-collisions',
        'module-2-digital-twin-sim/chapter-3-high-fidelity-rendering-hri',
        'module-2-digital-twin-sim/chapter-4-simulating-sensors-lidar-depth-imu',
      ],
    },
  ],

  // Manual sidebar for Isaac module
  isaacIntegrationSidebar: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac-integration/chapter-1-advanced-perception-training',
        'module-3-isaac-integration/chapter-2-isaac-sim-simulation-data',
        'module-3-isaac-integration/chapter-3-isaac-ros-vslam-navigation',
        'module-3-isaac-integration/chapter-4-nav2-bipedal-path-planning',
      ],
    },
  ],

  // Manual sidebar for Vision-Language-Action (VLA) module
  vlaIntegrationSidebar: [
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla-integration/chapter-1-llms-robotics-convergence',
        'module-4-vla-integration/chapter-2-voice-to-action-whisper',
        'module-4-vla-integration/chapter-3-cognitive-planning-llms',
        'module-4-vla-integration/chapter-4-capstone-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
