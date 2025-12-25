import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          Physical AI and Humanoid Robotics
        </Heading>
        <p className={styles.heroSubtitle}>
          A comprehensive guide to building intelligent robots that interact with the physical world
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/textbook/">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🤖 Foundations</h3>
              <p>
                Master the fundamentals of Physical AI, sensors, ROS 2, and URDF for robot modeling
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🎮 Simulation & Training</h3>
              <p>
                Learn Unity ML-Agents, Isaac Sim, and reinforcement learning for robot training
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🧠 AI Integration</h3>
              <p>
                Implement vision-language-action models and conversational AI for intelligent robots
              </p>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>📍 Navigation</h3>
              <p>
                Build autonomous navigation systems with path planning and SLAM
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🏗️ Capstone Project</h3>
              <p>
                Integrate everything into a fully functional voice-controlled robot assistant
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>💻 Hands-On</h3>
              <p>
                Practical exercises, code examples, and real-world implementations
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Comprehensive textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
