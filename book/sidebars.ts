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
  bookTutorial: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'book-tutorial/module-1/index',
        'book-tutorial/module-1/intro',
        'book-tutorial/module-1/python-integration',
        'book-tutorial/module-1/humanoid-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'book-tutorial/module-2/index',
        'book-tutorial/module-2/intro',
        'book-tutorial/module-2/chapter1-physics-simulation',
        'book-tutorial/module-2/chapter2-unity-visualization',
        'book-tutorial/module-2/chapter3-digital-twin-integration',
        'book-tutorial/module-2/data-model/data-model',
        'book-tutorial/module-2/integration/gazebo-unity-integration',
        'book-tutorial/module-2/physics/physics-simulation',
        'book-tutorial/module-2/quickstart/digital-twin-quickstart',
        'book-tutorial/module-2/sensors/sensor-simulation',
        'book-tutorial/module-2/unity/unity-visualization',
        'book-tutorial/module-2/validation/validation-plan',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'book-tutorial/module-3/index',
        'book-tutorial/module-3/intro',
        'book-tutorial/module-3/isaac-ros',
        'book-tutorial/module-3/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'book-tutorial/module-4/index',
        'book-tutorial/module-4/intro',
        'book-tutorial/module-4/voice-to-action',
        'book-tutorial/module-4/cognitive-planning',
        'book-tutorial/module-4/capstone-autonomous-humanoid',
      ],
    },
  ],

};

export default sidebars;
