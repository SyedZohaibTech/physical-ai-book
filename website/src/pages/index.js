import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className={clsx('hero__title', styles.heroTitle)}>{siteConfig.title}</h1>
        <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
          {siteConfig.tagline}
        </p>
        <p className={styles.heroDescription}>
          Learn how to build intelligent robots that interact with the physical world through simulation, perception, and control systems.
        </p>
        <p className={styles.heroDescription}>
          Master the technologies behind humanoid robotics including ROS 2, Gazebo, Unity, and NVIDIA Isaac.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ title, icon, description }) {
  return (
    <div className={clsx('col col--3', styles.featureCard)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function Features() {
  const features = [
    {
      title: 'ROS 2 Fundamentals',
      icon: '🤖',
      description: 'Learn the Robot Operating System 2, the backbone of modern robotics applications.',
    },
    {
      title: 'Gazebo & Unity Simulation',
      icon: '🎮',
      description: 'Master physics-based simulation environments for testing and validating robot behaviors.',
    },
    {
      title: 'NVIDIA Isaac Platform',
      icon: '⚡',
      description: 'Explore AI-powered robotics with NVIDIA Isaac for perception and navigation.',
    },
    {
      title: 'Vision-Language-Action',
      icon: '🗣️',
      description: 'Implement systems that understand and respond to human commands in real-world environments.',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <FeatureCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}

function AboutBook() {
  return (
    <section className={styles.aboutSection}>
      <div className="container">
        <h2 className={styles.aboutTitle}>About This Textbook</h2>
        <div className="row">
          <div className="col col--4">
            <p className={styles.aboutParagraph}>
              This comprehensive guide covers the essential technologies and concepts needed to build intelligent, embodied systems. 
              You'll learn through hands-on examples and real-world applications.
            </p>
          </div>
          <div className="col col--4">
            <p className={styles.aboutParagraph}>
              Each module includes practical exercises, code examples, and simulation scenarios that reinforce theoretical concepts 
              with tangible implementations you can experiment with.
            </p>
          </div>
          <div className="col col--4">
            <p className={styles.aboutParagraph}>
              By the end of this textbook, you'll have the skills to develop sophisticated humanoid robots capable of perceiving, 
              reasoning about, and interacting with the physical world.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function CustomFooter() {
  return (
    <footer className={clsx('footer', styles.footer)}>
      <div className="container">
        <div className={styles.footerContent}>
          <div className={styles.copyright}>
            &copy; {new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook
          </div>
          <div className={styles.authorAttribution}>
            Created by <span className={styles.authorName}>Syed Zohaib</span>
          </div>
        </div>
      </div>
    </footer>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics textbook - Master the Future of Embodied Intelligence">
      <HomepageHeader />
      <main>
        <Features />
        <AboutBook />
      </main>
      <CustomFooter />
    </Layout>
  );
}