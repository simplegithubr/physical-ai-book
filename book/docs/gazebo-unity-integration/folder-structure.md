---
sidebar_position: 2
---

# Recommended Folder Structure for Docusaurus Books

## Overview

Proper folder organization is essential for maintaining scalable and navigable documentation, especially for complex technical subjects like Gazebo-Unity integration. This guide presents recommended folder structures for organizing educational content in Docusaurus-based books.

## Table of Contents

- [Basic Docusaurus Structure](#basic-docusaurus-structure)
- [Educational Content Organization](#educational-content-organization)
- [Advanced Organization Patterns](#advanced-organization-patterns)
- [File Naming Conventions](#file-naming-conventions)
- [Navigation and Linking Strategies](#navigation-and-linking-strategies)
- [Content Maintenance Practices](#content-maintenance-practices)

## Basic Docusaurus Structure

### Default Directory Layout

Docusaurus follows a standardized structure that should be preserved:

```
my-book/
├── docs/
│   ├── intro.md
│   ├── tutorial-basics/
│   │   ├── _category_.json
│   │   ├── create-a-document.md
│   │   └── ...
│   └── ...
├── src/
├── static/
├── docusaurus.config.ts
├── sidebars.ts
└── package.json
```

### Core Directories

- **`docs/`**: Main documentation content
- **`src/`**: Custom React components and CSS
- **`static/`**: Static assets (images, PDFs, etc.)
- **`blog/`**: Blog posts and articles

## Educational Content Organization

### Pattern 1: Topic-Based Organization

Group content by subject matter:

```
docs/
├── gazebo-unity-integration/
│   ├── index.md
│   ├── architecture/
│   │   ├── overview.md
│   │   ├── communication-patterns.md
│   │   └── performance-optimization.md
│   ├── sensor-simulation/
│   │   ├── lidar-simulation.md
│   │   ├── depth-camera.md
│   │   ├── imu-simulation.md
│   │   └── fusion-techniques.md
│   ├── educational-patterns/
│   │   ├── course-structures.md
│   │   ├── learning-activities.md
│   │   └── assessment-methods.md
│   └── implementation-guides/
│       ├── setup-procedures.md
│       ├── troubleshooting.md
│       └── best-practices.md
```

### Pattern 2: Audience-Based Organization

Segment content by target audience:

```
docs/
├── beginners/
│   ├── getting-started.md
│   ├── basic-concepts.md
│   └── first-simulation.md
├── intermediate/
│   ├── advanced-configuration.md
│   ├── sensor-integration.md
│   └── performance-tuning.md
├── advanced/
│   ├── custom-plugins.md
│   ├── optimization-techniques.md
│   └── research-applications.md
└── educators/
    ├── curriculum-planning.md
    ├── classroom-setup.md
    └── assessment-tools.md
```

### Pattern 3: Learning Path Organization

Structure content as progressive learning paths:

```
docs/
├── foundations/
│   ├── robotics-basics.md
│   ├── simulation-concepts.md
│   └── platform-comparison.md
├── setup/
│   ├── environment-configuration.md
│   ├── toolchain-installation.md
│   └── initial-verification.md
├── basics/
│   ├── simple-navigation.md
│   ├── sensor-interaction.md
│   └── basic-control.md
├── intermediate/
│   ├── multi-sensor-systems.md
│   ├── algorithm-implementation.md
│   └── performance-optimization.md
└── advanced/
    ├── research-topics.md
    ├── custom-development.md
    └── capstone-projects.md
```

## Advanced Organization Patterns

### Pattern 1: Component-Based Organization

Organize by functional components:

```
docs/
├── platform-components/
│   ├── gazebo-setup.md
│   ├── unity-integration.md
│   ├── communication-layer.md
│   └── control-interface.md
├── sensor-systems/
│   ├── lidar-system.md
│   ├── camera-system.md
│   ├── imu-system.md
│   └── fusion-system.md
├── educational-tools/
│   ├── curriculum-builder.md
│   ├── assessment-tools.md
│   ├── progress-tracking.md
│   └── collaboration-features.md
└── deployment/
    ├── cloud-deployment.md
    ├── local-deployment.md
    ├── container-deployment.md
    └── scaling-strategies.md
```

### Pattern 2: Use Case-Based Organization

Organize by specific applications:

```
docs/
├── mobile-robotics/
│   ├── differential-drive.md
│   ├── navigation-algorithms.md
│   └── slam-techniques.md
├── manipulation/
│   ├── arm-control.md
│   ├── grasping-strategies.md
│   └── task-planning.md
├── aerial-vehicles/
│   ├── quadrotor-dynamics.md
│   ├── flight-control.md
│   └── aerial-navigation.md
├── underwater-vehicles/
│   ├── fluid-dynamics.md
│   ├── pressure-effects.md
│   └── underwater-navigation.md
└── swarm-robotics/
    ├── coordination-algorithms.md
    ├── communication-protocols.md
    └── emergent-behaviors.md
```

## File Naming Conventions

### General Guidelines

Follow consistent naming patterns for maintainability:

#### Documentation Files
- Use lowercase with hyphens: `sensor-simulation.md`
- Descriptive but concise: `performance-optimization.md` not `perf-opt.md`
- Avoid special characters: `lidar-simulation.md` not `lidar'sim.md`

#### Category Indexes
- Use `index.md` for category homepages
- Include `_category_.json` for configuration
- Maintain consistent metadata structure

### Naming Patterns

| Purpose | Pattern | Example |
|---------|---------|---------|
| Basic pages | `descriptive-topic-name.md` | `sensor-simulation.md` |
| Tutorials | `tutorial-topic-name.md` | `tutorial-lidar-setup.md` |
| Reference | `reference-topic-name.md` | `reference-api-spec.md` |
| Guides | `guide-topic-name.md` | `guide-performance-tuning.md` |
| Concepts | `concept-topic-name.md` | `concept-fusion-theory.md` |

## Navigation and Linking Strategies

### Sidebar Organization

Configure `sidebars.ts` for logical navigation:

```typescript
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'gazebo-unity-integration/index',
    {
      type: 'category',
      label: 'Architecture',
      items: [
        'gazebo-unity-integration/architecture/overview',
        'gazebo-unity-integration/architecture/communication-patterns',
        'gazebo-unity-integration/architecture/performance-optimization'
      ],
    },
    {
      type: 'category',
      label: 'Sensor Simulation',
      items: [
        'gazebo-unity-integration/sensor-simulation/lidar-simulation',
        'gazebo-unity-integration/sensor-simulation/depth-camera',
        'gazebo-unity-integration/sensor-simulation/imu-simulation',
        'gazebo-unity-integration/sensor-simulation/fusion-techniques'
      ],
    },
    // ... other categories
  ],
};
```

### Internal Linking

Maintain consistent linking practices:

#### Relative Links
```markdown
[LiDAR Simulation](./sensor-simulation/lidar-simulation.md)
[Performance Optimization](../architecture/performance-optimization.md)
```

#### Absolute Links
```markdown
[Getting Started](/docs/gazebo-unity-integration/index)
[Tutorial Basics](/docs/tutorial-basics/create-a-document)
```

### Cross-Reference Patterns

Create meaningful connections between related topics:

- Link foundational concepts to advanced applications
- Connect theory with practical implementations
- Provide "See Also" sections for related content
- Use breadcrumbs for hierarchical navigation

## Content Maintenance Practices

### Version Control

Structure content for easy maintenance:

#### Content Separation
- Keep conceptual content separate from implementation details
- Isolate platform-specific information
- Separate tutorials from reference material

#### Modular Design
- Small, focused files for easier updates
- Logical grouping of related concepts
- Consistent metadata patterns

### Documentation Standards

Maintain quality and consistency:

#### Metadata Requirements
```yaml
---
sidebar_position: 1
description: "Brief description for SEO and previews"
tags: [gazebo, unity, simulation, education]
---
```

#### Content Structure
- Consistent header hierarchy
- Standardized section organization
- Uniform formatting patterns

### Review and Update Processes

Establish maintenance workflows:

#### Regular Reviews
- Annual content audits
- Technology update synchronization
- Student feedback incorporation
- Industry practice alignment

#### Update Tracking
- Version control for documentation changes
- Change logs for major updates
- Deprecation notices for outdated content
- Migration guides for platform changes

## Implementation Guidelines

### Getting Started

1. **Choose Organization Pattern**: Select the most appropriate structure for your content
2. **Plan Navigation**: Map out logical pathways through content
3. **Establish Conventions**: Define naming and formatting standards
4. **Create Templates**: Develop standard formats for different content types

### Migration Strategies

When reorganizing existing content:

1. **Inventory Current Structure**: Catalog existing files and organization
2. **Map New Structure**: Plan the transition to new organization
3. **Update Links**: Ensure all internal links remain functional
4. **Test Navigation**: Verify all pathways work correctly

### Quality Assurance

Monitor and maintain structural integrity:

- **Link Validation**: Regular checking of broken links
- **Navigation Testing**: Ensure all pathways are accessible
- **Search Optimization**: Verify content is discoverable
- **Accessibility Compliance**: Maintain standards for all users

This organizational approach ensures that educational content remains discoverable, maintainable, and scalable as the documentation grows over time.