import React, { useState } from 'react';
import './styles.css';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div className={`chat-bot-container ${isOpen ? 'open' : ''}`}>
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Physical AI Assistant</h3>
            <button className="close-button" onClick={() => setIsOpen(false)}>×</button>
          </div>
          <div className="chat-messages">
            <p>Hello! I'm your Physical AI Assistant. How can I help you today?</p>
          </div>
          <div className="chat-input">
            <input type="text" placeholder="Type your message..." />
            <button>Send</button>
          </div>
        </div>
      ) : (
        <button className="chat-button" onClick={() => setIsOpen(true)}>
          💬 Chat
        </button>
      )}
    </div>
  );
};

export default ChatBot;
