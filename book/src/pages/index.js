import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Feature Data
const features = [
  {
    title: 'Physical AI',
    icon: '🤖',
    description: 'Bridge the gap between digital algorithms (LLMs) and physical actuation in the real world.',
    link: '/docs/intro'
  },
  {
    title: 'NVIDIA Isaac',
    icon: '🧠',
    description: 'Master photorealistic simulation and synthetic data generation for robust robot training.',
    link: '/docs/module-3-ai-robot-brain/isaac_sim_setup'
  },
  {
    title: 'ROS 2 Control',
    icon: '⚙️',
    description: 'The industry standard middleware (Robot Operating System) for controlling motors and sensors.',
    link: '/docs/module-1-ros-fundamentals/ros2_basics'
  },
  {
    title: 'Vision-Language',
    icon: '👁️',
    description: 'Integrate GPT-4o and VLA models to allow robots to understand and see their environment.',
    link: '/docs/module-4-vla/vla_intro'
  },
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero-banner">
      <div className="container">
        <h1 className="hero-title">Build the Body for the AI Mind</h1>
        <p className="hero-subtitle">
          The comprehensive guide to Physical AI, Humanoid Robotics, and Embodied Intelligence.
        </p>
        <div style={{ display: 'flex', gap: '20px', justifyContent: 'center', marginTop: '30px' }}>
          <Link className="cta-button" to="/docs/intro">
            Start Learning 🚀
          </Link>
          <Link className="cta-button" style={{ filter: 'grayscale(1)' }} to="https://github.com/panaversity">
            GitHub Repo
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics">
      
      <HomepageHeader />
      
      <main className="features-section">
        <div className="features-grid">
          {features.map((props, idx) => (
            <Link key={idx} to={props.link} style={{ textDecoration: 'none' }}>
              <div className="feature-card">
                <div className="feature-icon">{props.icon}</div>
                <h3 className="feature-title">{props.title}</h3>
                <p className="feature-desc">{props.description}</p>
              </div>
            </Link>
          ))}
        </div>
      </main>
      
    </Layout>
  );
}
