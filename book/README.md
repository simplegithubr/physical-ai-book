# Gazebo-Unity Integration for Educational Robotics

This Docusaurus-based documentation site provides comprehensive guidance on integrating Gazebo and Unity for educational robotics applications. The content covers best practices for combining Gazebo's accurate physics simulation with Unity's compelling visualization capabilities to create effective learning experiences.

## Educational Focus

This resource emphasizes:
- Progressive learning approaches for robotics education
- Realistic sensor simulation for educational purposes
- Performance optimization for classroom environments
- Scalable architecture for multiple simultaneous users

## Table of Contents

The documentation covers the following key areas:

1. **Gazebo-Unity Integration**: Overview of integration approaches and architecture
2. **Sensor Simulation**: Detailed coverage of sensor simulation across both platforms
3. **Educational Patterns**: Pedagogical approaches and course structure patterns
4. **Folder Structure**: Recommendations for organizing educational content
5. **Best Practices Summary**: Key takeaways and implementation guidance

## Target Audience

- Robotics educators and instructors
- Educational technologists
- Students studying robotics integration
- Researchers in educational robotics

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Contributing

Educators and practitioners are encouraged to contribute improvements and additional content to enhance this resource for the educational robotics community.
