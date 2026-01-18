import React from 'react';
import ResponseRenderer from './ResponseRenderer';

const MessageList = ({ messages }) => {
  return (
    <div
      className="chat-messages"
      aria-live="polite"
      aria-relevant="additions"
    >
      {messages.map((message) => (
        <div
          key={message.id}
          className={`message ${message.sender}`}
          role="log"
          aria-label={`${message.sender} message`}
        >
          <div className="message-content">
            {message.isError ? (
              <div className="error-message">
                {message.text}
              </div>
            ) : message.sender === 'bot' ? (
              <ResponseRenderer
                content={message.text}
                sources={message.sources || []}
                confidence={message.confidence}
              />
            ) : (
              <div>{message.text}</div>
            )}
          </div>
        </div>
      ))}
    </div>
  );
};

export default MessageList;