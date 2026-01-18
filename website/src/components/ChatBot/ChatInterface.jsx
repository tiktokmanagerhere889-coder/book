import React, { useState, useEffect } from 'react';
import MessageList from './MessageList';
import MessageInput from './MessageInput';
import { sendMessage, getSessionId } from '@site/src/utils/chatService';

const ChatInterface = ({ onClose }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);

  // Initialize session and load any existing messages from localStorage
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const newSessionId = getSessionId();
      setSessionId(newSessionId);

      // Load existing messages from localStorage if any
      const savedMessages = localStorage.getItem('chatbot_messages');
      if (savedMessages) {
        try {
          const parsedMessages = JSON.parse(savedMessages);
          setMessages(parsedMessages);
        } catch (e) {
          console.error('Error parsing saved messages:', e);
          setMessages([{ id: 1, text: 'Hello! I\'m your book assistant. How can I help you today?', sender: 'bot', timestamp: new Date() }]);
        }
      } else {
        setMessages([{ id: 1, text: 'Hello! I\'m your book assistant. How can I help you today?', sender: 'bot', timestamp: new Date() }]);
      }
    }
  }, []);

  // Add reconnection logic when component mounts
  useEffect(() => {
    // This is where we could add additional reconnection logic if needed
    // For example, if we had WebSocket connections or needed to re-establish server connections
    console.log('ChatInterface mounted, session restored');

    // Cleanup function if needed
    return () => {
      // Any cleanup code when component unmounts
    };
  }, []);

  // Save messages to localStorage whenever they change
  useEffect(() => {
    if (typeof window !== 'undefined' && messages.length > 0) {
      localStorage.setItem('chatbot_messages', JSON.stringify(messages));
    }
  }, [messages]);

  const handleSendMessage = async (messageText) => {
    if (!messageText.trim()) return;

    // Add user message to the list
    const userMessage = {
      id: Date.now(),
      text: messageText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Call the backend API to get the response
      const response = await sendMessage(messageText, sessionId);

      // Update session ID if it changed
      if (response.sessionId && response.sessionId !== sessionId) {
        setSessionId(response.sessionId);
        localStorage.setItem('chatbot_session_id', response.sessionId);
      }

      // Add bot response to the list
      const botMessage = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        sources: response.sources || [],
        confidence: response.confidence,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      setError(err.message || 'Sorry, I encountered an error. Please try again.');

      // Add error message to the list
      const errorMessage = {
        id: Date.now() + 1,
        text: err.message || 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        isError: true,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearError = () => {
    setError(null);
  };

  return (
    <div className="chat-interface">
      <MessageList messages={messages} />
      <MessageInput
        onSendMessage={handleSendMessage}
        isLoading={isLoading}
        error={error}
        onErrorClear={clearError}
      />
    </div>
  );
};

export default ChatInterface;