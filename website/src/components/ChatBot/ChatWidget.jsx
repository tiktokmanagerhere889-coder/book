import React, { useState, useEffect } from 'react';
import ChatInterface from './ChatInterface';
import '../../css/chatbot.css';

const ChatWidget = () => {
  const [isMinimized, setIsMinimized] = useState(true);
  const [isOpen, setIsOpen] = useState(false);
  const [isClient, setIsClient] = useState(false);

  // Initialize state from localStorage after component mounts
  useEffect(() => {
    setIsClient(true);

    if (typeof window !== 'undefined') {
      const savedMinimized = localStorage.getItem('chatbot_minimized');
      const savedOpen = localStorage.getItem('chatbot_open');

      setIsMinimized(savedMinimized ? JSON.parse(savedMinimized) : true);
      setIsOpen(savedOpen ? JSON.parse(savedOpen) : false);
    }
  }, []);

  // Save state to localStorage whenever it changes (only in browser)
  useEffect(() => {
    if (typeof window !== 'undefined' && isClient) {
      localStorage.setItem('chatbot_minimized', JSON.stringify(isMinimized));
      localStorage.setItem('chatbot_open', JSON.stringify(isOpen));
    }
  }, [isMinimized, isOpen, isClient]);

  const toggleChat = () => {
    if (isMinimized) {
      setIsMinimized(false);
      setIsOpen(true);
    } else {
      setIsOpen(!isOpen);
    }
  };

  const minimizeChat = () => {
    setIsOpen(false);
    setTimeout(() => setIsMinimized(true), 300);
  };

  const handleKeyDown = (e) => {
    // Toggle chat with Ctrl/Cmd + Shift + K
    if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key === 'K') {
      e.preventDefault();
      toggleChat();
    }
  };

  // Add keyboard shortcut listener
  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  return (
    <div className="chatbot-container" role="complementary" aria-label="Book Assistant Chat">
      {isOpen || !isMinimized ? (
        <div
          className={`chat-widget ${isMinimized ? 'minimized' : ''}`}
          role="region"
          aria-label="Chat interface"
          tabIndex={-1}
        >
          <div
            className={`chat-header ${isMinimized ? 'minimized' : ''}`}
            onClick={minimizeChat}
            role="button"
            tabIndex={0}
            aria-expanded={!isMinimized}
            aria-label={isMinimized ? "Expand chat" : "Minimize chat"}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                minimizeChat();
              }
            }}
          >
            {!isMinimized && (
              <>
                <h3 className="chat-title" tabIndex={-1}>Book Assistant</h3>
                <button
                  className="chat-toggle"
                  onClick={(e) => { e.stopPropagation(); minimizeChat(); }}
                  aria-label="Minimize chat"
                  tabIndex={0}
                >
                  −
                </button>
              </>
            )}
            {isMinimized && <span aria-hidden="true">💬</span>}
          </div>
          {!isMinimized && isOpen && <ChatInterface onClose={() => setIsOpen(false)} />}
        </div>
      ) : (
        <div
          className="chat-widget minimized"
          role="button"
          tabIndex={0}
          aria-label="Open chat"
          aria-expanded="false"
          onClick={toggleChat}
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              toggleChat();
            }
          }}
        >
          <div className="chat-header minimized">
            <span aria-hidden="true">💬</span>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;