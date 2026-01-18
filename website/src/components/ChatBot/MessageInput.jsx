import React, { useState } from 'react';
import ErrorDisplay from './ErrorDisplay';

const MessageInput = ({ onSendMessage, isLoading, error, onErrorClear }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
      setInputValue('');
      if (onErrorClear) {
        onErrorClear();
      }
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (!isLoading) {
        handleSubmit(e);
      }
    }
  };

  const handleRetry = () => {
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
    }
  };

  const handleDismiss = () => {
    if (onErrorClear) {
      onErrorClear();
    }
  };

  return (
    <div className="chat-input-area">
      <form onSubmit={handleSubmit} style={{ width: '100%' }}>
        <div style={{ display: 'flex', gap: '8px' }}>
          <textarea
            className="chat-input"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyPress}
            placeholder="Ask a question about the book..."
            disabled={isLoading}
            rows={1}
            aria-label="Type your message"
            role="textbox"
            aria-multiline="true"
          />
          <button
            type="submit"
            className="send-button"
            disabled={isLoading || !inputValue.trim()}
            aria-label={isLoading ? "Sending message" : "Send message"}
          >
            {isLoading ? (
              <span className="spinner" aria-label="Sending"></span>
            ) : (
              <span>➤</span>
            )}
          </button>
        </div>
      </form>
      {error && (
        <ErrorDisplay
          error={error}
          onRetry={handleRetry}
          onDismiss={handleDismiss}
        />
      )}
    </div>
  );
};

export default MessageInput;