import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Default wrapper for the entire app
const Root = ({children}) => {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          const ChatWidget = require('@site/src/components/ChatBot/ChatWidget').default;
          return <ChatWidget />;
        }}
      </BrowserOnly>
    </>
  );
};

export default Root;