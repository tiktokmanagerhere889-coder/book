import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Custom Root component to handle browser-specific functionality safely
// Ensures no browser APIs are accessed during SSG (static site generation)
export default function Root({children}) {
  return (
    <BrowserOnly fallback={<>{children}</>}>
      {() => {
        // This only executes in the browser environment
        // Any browser-specific code should be placed here
        return <>{children}</>;
      }}
    </BrowserOnly>
  );
}