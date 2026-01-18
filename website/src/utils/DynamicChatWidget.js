import dynamic from 'next/dynamic';

// Create a dynamically imported ChatWidget that only loads on the client-side
const DynamicChatWidget = dynamic(
  () => import('./components/ChatBot/ChatWidget'),
  {
    ssr: false, // Disable server-side rendering for this component
    loading: () => (
      <div
        className="chatbot-container"
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: '1000',
          display: 'flex',
          flexDirection: 'column'
        }}
      >
        <div
          className="chat-widget minimized"
          style={{
            width: '60px',
            height: '60px',
            borderRadius: '12px',
            boxShadow: '0 10px 50px rgba(0, 0, 0, 0.3)',
            backgroundColor: 'white',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            cursor: 'pointer',
            border: '1px solid #e5e7eb'
          }}
        >
          <div
            className="chat-header minimized"
            style={{
              width: '100%',
              height: '100%',
              display: 'flex',
              justifyContent: 'center',
              alignItems: 'center'
            }}
          >
            <span className="chat-icon">💬</span>
          </div>
        </div>
      </div>
    ),
  }
);

export default DynamicChatWidget;