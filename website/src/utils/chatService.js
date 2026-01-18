import axios from 'axios';

// Determine the API base URL based on environment
const getApiBaseUrl = () => {
  if (typeof window !== 'undefined') {
    // Client-side: use relative path or environment variable
    return process.env.REACT_APP_API_BASE_URL || '';
  }
  // Server-side: use environment variable or default
  return process.env.API_BASE_URL || '';
};

const API_BASE_URL = getApiBaseUrl();

/**
 * Send a message to the chat API and receive a response
 * @param {string} message - The user's message
 * @param {string} sessionId - The current session ID
 * @returns {Promise<Object>} The API response containing the bot's reply and metadata
 */
export const sendMessage = async (message, sessionId = null) => {
  try {
    // Prepare the request payload
    const requestData = {
      query: message,
    };

    // Include session ID if available
    if (sessionId) {
      requestData.session_id = sessionId;
    }

    const response = await axios.post(`${API_BASE_URL}/api/chat/query`, requestData, {
      headers: {
        'Content-Type': 'application/json'
      },
      timeout: 30000 // 30 second timeout
    });

    // Extract and format the response data
    const responseData = response.data;

    // Return the response data with standardized format
    return {
      response: responseData.response || responseData.answer || responseData.content || responseData,
      sources: responseData.sources || responseData.references || responseData.citations || [],
      confidence: responseData.confidence || responseData.confidence_score || responseData.confidenceScore || null,
      sessionId: responseData.session_id || responseData.sessionId || sessionId
    };
  } catch (error) {
    console.error('Error sending message:', error);

    // Handle different types of errors
    if (error.response) {
      // Server responded with error status
      const errorMsg = error.response.data?.detail || error.response.data?.message || error.response.statusText;
      throw new Error(`Server error: ${error.response.status} - ${errorMsg}`);
    } else if (error.request) {
      // Request was made but no response received
      throw new Error('Network error: Unable to reach the server. Please check your connection.');
    } else {
      // Something else happened
      throw new Error(error.message || 'An unexpected error occurred');
    }
  }
};

/**
 * Initialize a new chat session
 * @returns {Promise<string>} The session ID
 */
export const initializeSession = async () => {
  try {
    // For now, we'll use a simple approach. In a real implementation,
    // this would call an API endpoint to create a new session
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  } catch (error) {
    console.error('Error initializing session:', error);
    throw error;
  }
};

/**
 * Get the current session ID from localStorage or create a new one
 * @returns {string} The session ID
 */
export const getSessionId = () => {
  let sessionId = localStorage.getItem('chatbot_session_id');
  if (!sessionId) {
    sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem('chatbot_session_id', sessionId);
  }
  return sessionId;
};

/**
 * Clear the current session data
 */
export const clearSession = () => {
  localStorage.removeItem('chatbot_session_id');
  localStorage.removeItem('chatbot_messages');
  localStorage.removeItem('chatbot_minimized');
  localStorage.removeItem('chatbot_open');
};

/**
 * Get message history from localStorage
 * @returns {Array} Array of message objects
 */
export const getMessageHistory = () => {
  const savedMessages = localStorage.getItem('chatbot_messages');
  if (savedMessages) {
    try {
      return JSON.parse(savedMessages);
    } catch (e) {
      console.error('Error parsing saved messages:', e);
      return [];
    }
  }
  return [];
};

/**
 * Save message history to localStorage
 * @param {Array} messages - Array of message objects to save
 */
export const saveMessageHistory = (messages) => {
  try {
    localStorage.setItem('chatbot_messages', JSON.stringify(messages));
  } catch (e) {
    console.error('Error saving messages:', e);
  }
};

/**
 * Update session with new message
 * @param {Object} message - The message object to add to history
 */
export const addMessageToHistory = (message) => {
  const messages = getMessageHistory();
  messages.push(message);
  saveMessageHistory(messages);
};