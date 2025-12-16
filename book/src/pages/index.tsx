import type {ReactNode} from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
      </div>
    </header>
  );
}

type ModuleCard = {
  title: string;
  description: string;
};

function useModules(): ModuleCard[] {
  return [
    {
      title: 'Module 1: The Robotic Nervous System (ROS 2)',
      description: 'Learn about the robotic nervous system using ROS 2 framework',
    },
    {
      title: 'Module 2: Digital Twin (Gazebo & Unity)',
      description: 'Explore digital twin technology with Gazebo and Unity integration',
    },
    {
      title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      description: 'Build intelligent robot brains using NVIDIA Isaac platform',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Implement vision-language-action models for robotic applications',
    },
  ];
}

function ModuleCard({ title, description }: ModuleCard) {
  return (
    <div className="col col--3">
      <div className="card">
        <div className="card__body">
          <h3>{title}</h3>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const modules = useModules();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course - Complete Learning Path">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className="container padding-vert--lg">
            <div className="row">
              {modules.map((module, index) => (
                <ModuleCard key={index} {...module} />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
