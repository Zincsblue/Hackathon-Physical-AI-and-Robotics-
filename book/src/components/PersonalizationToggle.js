import React, { useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const PersonalizationToggle = ({ children, level }) => {
  // Default state
  const [currentMode, setCurrentMode] = useState('beginner');

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      // 1. On load, read the saved preference from LocalStorage
      const savedMode = localStorage.getItem('user-skill-level') || 'beginner';
      setCurrentMode(savedMode);

      // 2. Listen for a custom event (so all buttons update at once)
      const handleLevelChange = () => {
        const newMode = localStorage.getItem('user-skill-level');
        setCurrentMode(newMode);
      };

      window.addEventListener('skill-level-change', handleLevelChange);

      // Cleanup listener
      return () => {
        window.removeEventListener('skill-level-change', handleLevelChange);
      };
    }
  }, []);

  const toggleMode = () => {
    const newMode = currentMode === 'beginner' ? 'advanced' : 'beginner';
    
    if (ExecutionEnvironment.canUseDOM) {
      // Save to storage
      localStorage.setItem('user-skill-level', newMode);
      // Trigger the event so ALL components on the page update instantly
      window.dispatchEvent(new Event('skill-level-change'));
    }
  };

  // LOGIC: Check if this component should be visible
  const isVisible = level === currentMode;

  return (
    <div style={{
      marginBottom: '20px',
      border: isVisible ? '1px solid #e0e0e0' : '1px dashed #ccc',
      borderRadius: '8px',
      overflow: 'hidden', // Keeps corners neat
      backgroundColor: isVisible ? '#1c1a1aff' : '#1c1a1aff',
      transition: 'all 0.3s ease'
    }}>
      
      {/* HEADER BAR (Always visible) */}
      <div style={{
        backgroundColor: currentMode === 'beginner' ? '#e3f2fd' : '#e8f5e9', // Blue for Beginner, Green for Advanced
        padding: '10px 15px',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        borderBottom: '1px solid #ddd'
      }}>
        <span style={{fontWeight: 'bold', fontSize: '14px', color: '#333'}}>
          {currentMode === 'beginner' ? '👶 Mode: BEGINNER' : '🚀 Mode: ADVANCED'}
        </span>
        
        <button 
          onClick={toggleMode}
          style={{
            cursor: 'pointer',
            padding: '5px 12px',
            fontSize: '12px',
            backgroundColor: '#075b7cff',
            border: '1px solid #ccc',
            borderRadius: '4px',
            boxShadow: '0 1px 2px rgba(144, 133, 133, 0.1)'
          }}
          
        >
          Switch to {currentMode === 'beginner' ? 'Advanced' : 'Beginner'}
        </button>
      </div>

      {/* CONTENT AREA */}
      <div style={{ padding: '20px' }}>
        {isVisible ? (
          <div>
            {/* Show the content */}
            {children} 
          </div>
        ) : (
          /* Show a "Hidden" message instead */
          <div style={{
            color: '#0f0303ff', 
            fontStyle: 'italic', 
            textAlign: 'center',
            fontSize: '0.9rem'
          }}>
            This content is hidden because it is for <strong>{level.toUpperCase()}</strong> users.
            <br/>
            (Switch modes above to view this)
          </div>
        )}
      </div>
    </div>
  );
};

export default PersonalizationToggle;