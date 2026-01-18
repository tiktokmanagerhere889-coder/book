import React from 'react';

// Simple function to sanitize HTML by removing potentially dangerous tags/attributes
const sanitizeHTML = (unsafeString) => {
  if (!unsafeString) return '';

  // Remove potentially dangerous HTML tags
  let sanitized = unsafeString.replace(/<(script|iframe|object|embed|form|input|button|link|meta|base)[^>]*>.*?<\/\1>|<(script|iframe|object|embed|form|input|button|link|meta|base)[^>]*\/?>/gi, '');

  // Remove potentially dangerous attributes
  sanitized = sanitized.replace(/(on\w+)=("[^"]*"|'[^']*'|[\w\-.]+)(?=\s|>)/gi, '');

  // Decode HTML entities safely
  const textarea = document.createElement('textarea');
  textarea.innerHTML = sanitized;
  return textarea.textContent || textarea.innerText || '';
};

const ResponseRenderer = ({ content, sources = [], confidence = null, timestamp = null }) => {
  // Safely render content with basic markdown-like formatting
  const renderContent = (text) => {
    if (!text) return null;

    // Sanitize the content to prevent XSS
    const sanitizedText = sanitizeHTML(text);

    // Basic text processing for line breaks
    const paragraphs = sanitizedText.split('\n').filter(paragraph => paragraph.trim() !== '');

    return (
      <div>
        {paragraphs.map((paragraph, index) => (
          <p key={index} style={{ marginBottom: '0.5em' }}>{paragraph}</p>
        ))}
      </div>
    );
  };

  return (
    <div className="response-content">
      <div className="response-text">
        {renderContent(content)}
      </div>

      {(sources && sources.length > 0) && (
        <div className="citations" aria-label="Supporting citations">
          <strong>Citations:</strong>
          <div className="citation-list">
            {sources.map((source, index) => {
              // Sanitize source data as well
              const safeTitle = source.title ? sanitizeHTML(source.title) : null;
              const safeUrl = source.url ? sanitizeHTML(source.url) : null;

              return (
                <span
                  key={index}
                  className="citation"
                  title={safeTitle || safeUrl || source.page_content?.substring(0, 100) || `Source ${index + 1}`}
                >
                  {safeTitle || safeUrl || `Source ${index + 1}`}
                </span>
              );
            })}
          </div>
        </div>
      )}

      {confidence !== null && (
        <div
          className="confidence-info"
          title={`Confidence Score: ${confidence}`}
          aria-label={`Confidence: ${Math.round(confidence * 100)}%`}
        >
          <small>Confidence: {Math.round(confidence * 100)}%</small>
        </div>
      )}

      {timestamp && (
        <div className="timestamp" aria-label={`Sent at ${new Date(timestamp).toLocaleTimeString()}`}>
          <small>{new Date(timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</small>
        </div>
      )}
    </div>
  );
};

export default ResponseRenderer;