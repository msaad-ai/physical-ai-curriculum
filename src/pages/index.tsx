import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.mainTitle}>
              Physical AI & Humanoid Robotics
            </Heading>
            <p className={styles.subtitle}>
              A complete multi-module curriculum covering ROS 2, simulation, computer vision,
              deep learning, humanoid robotics, and VLA systems — built for next-generation
              robotics developers.
            </p>
            <div className={styles.heroButtons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Learning
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src="img/physical-ai-book-cover.svg"
              alt="Humanoid Robot"
              className={styles.robotImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function ModulesSection() {
  const modules = [
    {
      title: 'Module 1 — The Robotic Nervous System (ROS 2)',
      path: '/docs/module1/intro',
      description: 'Build a strong foundation in ROS 2 concepts and development.'
    },
    {
      title: 'Module 2 — Digital Twin (Gazebo & Unity)',
      path: '/docs/module2/intro',
      description: 'Master simulation environments and digital twin technologies.'
    },
    {
      title: 'Module 3 — AI-Robot Brain (NVIDIA Isaac)',
      path: '/docs/module3/intro',
      description: 'Implement perception and navigation systems for intelligent robots.'
    },
    {
      title: 'Module 4 — Vision-Language-Action (VLA) Systems',
      path: '/docs/module4',
      description: 'Integrate vision, language, and action for advanced robotics.'
    }
  ];

  return (
    <section id="modules" className={styles.modulesSection}>
      <div className="container padding-vert--lg">
        <div className="text--center padding-bottom--xl">
          <Heading as="h2" className={styles.sectionTitle}>
            Course Modules
          </Heading>
          <p className={styles.sectionSubtitle}>
            Comprehensive curriculum designed for next-generation robotics developers
          </p>
        </div>
        <div className="row">
          {modules.map((module, index) => (
            <div key={index} className="col col--3 margin-bottom--lg">
              <div className={styles.moduleCard}>
                <h3 className={styles.moduleTitle}>
                  <Link to={module.path}>
                    {module.title}
                  </Link>
                </h3>
                <p className={styles.moduleDescription}>
                  {module.description}
                </p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A complete multi-module curriculum covering ROS 2, simulation, computer vision, deep learning, humanoid robotics, and VLA systems">
      <HomepageHeader />
      <main>
        <ModulesSection />
      </main>
    </Layout>
  );
}