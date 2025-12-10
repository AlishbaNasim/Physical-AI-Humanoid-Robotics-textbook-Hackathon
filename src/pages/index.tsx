import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">Master the Future of Robotics & AI Integration</p>
          <div className={styles.description}>
            <p>Discover how artificial intelligence meets physical reality in the most comprehensive guide to humanoid robotics.</p>
            <p>From ROS 2 fundamentals to advanced embodied intelligence, transform your understanding of AI systems in the physical world.</p>
          </div>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/chapter-1-introduction/overview">
              Start Reading
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics - A comprehensive guide to ROS 2 and humanoid robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
