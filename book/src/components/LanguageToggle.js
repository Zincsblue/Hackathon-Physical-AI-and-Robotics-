import React, { useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// --- TRANSLATION DICTIONARY ---
// This maps the English Menu items to Urdu
const menuTranslations = {
  "Welcome to the Future of Work": "Future of Work mein Khush Amdeed",
  "What is Physical AI?": "Physical AI Kya Hai?",
  "Course Overview": "Course Overview (Khulasa)",
  "Prerequisites & Hardware": "Zaroori Hardware",
  "Learning Outcomes": "Seekhnay Ke Nataij",
  "Let's Begin": "Chaliye Shuru Karte Hain",
  "Introduction to Physical AI & Humanoid Robotics": "Physical AI ka Taaruf"
};

const LanguageToggle = ({ children, lang, type = "wrapper" }) => {
  const [currentLang, setCurrentLang] = useState('en');

  // --- HELPER: FUNCTION TO TRANSLATE SIDEBAR ---
  const updateSidebar = (language) => {
    if (!ExecutionEnvironment.canUseDOM) return;

    // Find all links in the Right-Hand Table of Contents
    const tocLinks = document.querySelectorAll('.table-of-contents a');

    tocLinks.forEach(link => {
      const text = link.innerText.trim();
      
      if (language === 'ur') {
        // Switch to Urdu: Look up the English text in our dictionary
        if (menuTranslations[text]) {
          link.innerText = menuTranslations[text];
          link.setAttribute('data-original-text', text); // Save English for later
        }
      } else {
        // Switch to English: Restore the saved text or reverse lookup
        // Ideally, we look for the Urdu value in our object to find the English key
        const englishKey = Object.keys(menuTranslations).find(key => menuTranslations[key] === text);
        if (englishKey) {
          link.innerText = englishKey;
        } else if (link.getAttribute('data-original-text')) {
           // Fallback if we saved it
           link.innerText = link.getAttribute('data-original-text');
        }
      }
    });
  };

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      const savedLang = localStorage.getItem('site-language') || 'en';
      setCurrentLang(savedLang);
      
      // Update sidebar immediately on load
      setTimeout(() => updateSidebar(savedLang), 500); 

      const handleLangChange = () => {
        const newLang = localStorage.getItem('site-language');
        setCurrentLang(newLang);
        updateSidebar(newLang); // Update sidebar when event fires
      };

      window.addEventListener('language-change', handleLangChange);
      return () => window.removeEventListener('language-change', handleLangChange);
    }
  }, []);

  const toggleLanguage = () => {
    const newLang = currentLang === 'en' ? 'ur' : 'en';
    if (ExecutionEnvironment.canUseDOM) {
      localStorage.setItem('site-language', newLang);
      window.dispatchEvent(new Event('language-change'));
    }
  };

  // --- RENDER: BUTTON MODE ---
  if (type === 'controller') {
    return (
      <div style={{
        backgroundColor: currentLang === 'en' ? '#f1f8e9' : '#e0f2f1',
        padding: '15px',
        borderRadius: '8px',
        border: '1px solid #2e7d32',
        marginBottom: '20px',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        boxShadow: '0 2px 5px rgba(0,0,0,0.1)'
      }}>
        <span style={{ fontWeight: 'bold', color: '#2e7d32', fontSize: '16px' }}>
          {currentLang === 'en' ? '🇬🇧 Language: ENGLISH' : '🇵🇰 Language: ROMAN URDU'}
        </span>
        <button 
          onClick={toggleLanguage}
          style={{
            cursor: 'pointer',
            padding: '8px 16px',
            fontSize: '14px',
            fontWeight: 'bold',
            backgroundColor: '#25c2a0',
            color: 'white',
            border: 'none',
            borderRadius: '6px',
            boxShadow: '0 2px 4px rgba(0,0,0,0.2)'
          }}
        >
          {currentLang === 'en' ? 'Translate to Roman Urdu' : 'Switch back to English'}
        </button>
      </div>
    );
  }

  // --- RENDER: WRAPPER MODE ---
  if (lang === currentLang) {
    return <div className="fade-in">{children}</div>;
  }

  return null;
};

export default LanguageToggle;