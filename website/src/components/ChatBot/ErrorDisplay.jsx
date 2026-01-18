import React from 'react';

const ErrorDisplay = ({ error, onRetry, onDismiss }) => {
  if (!error) return null;

  // Determine error type and provide appropriate message
  let errorMessage = error;
  let errorType = 'general';

  if (error.includes('Network error')) {
    errorType = 'network';
    errorMessage = 'Unable to connect to the server. Please check your internet connection.';
  } else if (error.includes('Server error')) {
    errorType = 'server';
    errorMessage = 'The server encountered an error. Please try again later.';
  }

  return (
    <div
      className="error-display"
      role="alert"
      aria-live="assertive"
      style={{
        backgroundColor: '#fee2e2',
        color: '#dc2626',
        padding: '12px 16px',
        borderRadius: '8px',
        margin: '8px 0',
        fontSize: '14px',
        borderLeft: '4px solid #dc2626',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}
    >
      <div>
        <strong>Error:</strong> {errorMessage}
      </div>
      <div style={{ display: 'flex', gap: '8px' }}>
        {onRetry && (
          <button
            onClick={onRetry}
            style={{
              backgroundColor: '#dc2626',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 8px',
              cursor: 'pointer',
              fontSize: '12px'
            }}
            aria-label="Retry request"
          >
            Retry
          </button>
        )}
        {onDismiss && (
          <button
            onClick={onDismiss}
            style={{
              backgroundColor: '#9ca3af',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              padding: '4px 8px',
              cursor: 'pointer',
              fontSize: '12px'
            }}
            aria-label="Dismiss error"
          >
            Dismiss
          </button>
        )}
      </div>
    </div>
  );
};

export default ErrorDisplay;