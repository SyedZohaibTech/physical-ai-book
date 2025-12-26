import React from 'react';
import ChatBot from '@site/src/components/Chatbot';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
