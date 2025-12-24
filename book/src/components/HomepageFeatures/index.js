import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Physical AI',
    icon: '🦾',
    description: (
      <>
        Bridging the gap between digital algorithms and physical actuation.
        Control humanoid robots in the real world.
      </>
    ),
  },
  {
    title: 'NVIDIA Isaac',
    icon: '🧠',
    description: (
      <>
        Photorealistic simulation & synthetic data generation.
      </>
    ),
  },
  {
    title: 'ROS 2',
    icon: '🔗',
    description: (
      <>
        The standard middleware for robot control.
      </>
    ),
  },
  {
    title: 'VLA Models',
    icon: '👁️', 
    description: (
      <>
        Vision-Language-Action models that enable robots to reason and plan
        using visual inputs and natural language.
      </>
    ),
  },
];

function Feature({icon, title, description}) {
  return (
    // CHANGE: We switched from 'col--4' to 'col--3' so 4 cards fit in one row
    <div className={clsx('col col--3')}>
      <div className="feature-card" style={{
        padding: '1.5rem', 
        borderRadius: '10px', 
        backgroundColor: '#1a1a1a', 
        height: '100%', 
        border: '1px solid #333',
        marginBottom: '1rem' // Adds spacing on mobile
      }}>
        <div className="text--center" style={{fontSize: '3rem', marginBottom: '1rem'}}>
          {icon}
        </div>
        <div className="text--center padding-horiz--md">
          <h3 style={{color: 'white', fontSize: '1.2rem'}}>{title}</h3>
          <p style={{color: '#aaa', fontSize: '0.9rem'}}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features} style={{backgroundColor: '#000', padding: '4rem 0'}}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}