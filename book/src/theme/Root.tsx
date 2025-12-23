import React, { useState, ChangeEvent } from 'react';
import ChatBotWidget from '../components/ChatBotWidget';

// Custom Root component to wrap the entire app
export default function Root({children}: {children: React.ReactNode}) {
  const [selectedLanguage, setSelectedLanguage] = useState('en');

  const handleLanguageChange = (e: ChangeEvent<HTMLSelectElement>) => {
    const value = e.target.value;
    setSelectedLanguage(value);

    if (value === 'ur') {
      const url = window.location.href;
      window.location.href = `https://translate.google.com/translate?sl=en&tl=ur&hl=ur&u=${encodeURIComponent(url)}`;
    }
  };

  return (
    <>
      {/* Language switcher dropdown - appears on all pages */}
      <div style={{
        position: 'fixed',
        top: '20px',
        right: '20px',
        zIndex: 1000,
      }}>
        <select
          value={selectedLanguage}
          onChange={handleLanguageChange}
          style={{
            backgroundColor: '#166534', // Green background to match book theme
            color: 'white', // White text
            border: 'none',
            borderRadius: '6px',
            padding: '8px 12px',
            fontSize: '14px',
            cursor: 'pointer',
            fontWeight: '500',
            boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
            appearance: 'none', // Remove default dropdown arrow
            minWidth: '120px',
            textAlign: 'center',
          }}
          onMouseEnter={(e) => {
            (e.target as HTMLElement).style.backgroundColor = '#15803d'; // Darker green on hover
          }}
          onMouseLeave={(e) => {
            (e.target as HTMLElement).style.backgroundColor = '#166534'; // Original green
          }}
        >
          <option value="en">English</option>
          <option value="ur">اردو</option>
        </select>
      </div>

      {children}
      <ChatBotWidget />
    </>
  );
}