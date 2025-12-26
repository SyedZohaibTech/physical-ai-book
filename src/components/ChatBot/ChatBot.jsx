import React, { useState, useEffect, useRef } from 'react';
import './ChatBot.css';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, role: 'assistant', content: 'Hello! How can I help you today?' }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  // Effect to handle scrolling when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending messages
  const handleSend = async () => {
    // Check if input is empty before sending
    if (!input.trim()) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: input
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;

    // Clear input
    setInput('');
    setIsLoading(true);

    // Simulate a response after a short delay
    setTimeout(() => {
      const responses = [
        `I understand you said: "${currentInput}". How else can I assist you?`,
        `Thanks for your message: "${currentInput}". What else would you like to know?`,
        `I received your message: "${currentInput}". Feel free to ask anything else!`,
        `Interesting point about "${currentInput}". Can you tell me more?`,
        `I've noted your message: "${currentInput}". Ask me anything else!`
      ];
      
      const randomResponse = responses[Math.floor(Math.random() * responses.length)];
      
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: randomResponse
      };

      setMessages(prev => [...prev, assistantMessage]);
      setIsLoading(false);
    }, 1000);
  };

  // Handle pressing Enter key
  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // Toggle chat window open/closed
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Floating chat button */}
      <button className="chat-button" onClick={toggleChat} aria-label="Open chat">
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16.585L19.293 19.708C19.4777 19.8926 19.585 20.141 19.585 20.4C19.585 20.659 19.4777 20.9074 19.293 21.092C19.1083 21.2767 18.859 21.384 18.6 21.384C18.341 21.384 18.0927 21.2767 17.908 21.092L15 18.185V21C15 21.2652 14.8946 21.5196 14.7071 21.7071C14.5196 21.8946 14.2652 22 14 22H5C4.73478 22 4.48043 21.8946 4.29289 21.7071C4.10536 21.5196 4 21.2652 4 21V9C4 8.73478 4.10536 8.48043 4.29289 8.29289C4.48043 8.0196 4.73478 8 5 8H14C14.2652 8 14.5196 8.10536 14.7071 8.29289C14.8946 8.48043 15 8.73478 15 9V11.813L17.707 8.906C17.8917 8.72152 18.141 8.61446 18.4 8.61446C18.659 8.61446 18.9083 8.72152 19.093 8.906C19.2777 9.09069 19.385 9.34001 19.385 9.6C19.385 9.85999 19.2777 10.1093 19.093 10.294L16.415 13H19C19.5304 13 20.0391 13.2107 20.4142 13.5858C20.7893 13.9609 21 14.4696 21 15ZM13 17H6V19H13V17ZM13 13H6V15H13V13ZM13 9H6V11H13V9Z" fill="white"/>
        </svg>
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Physical AI Assistant</h3>
            <button className="close-button" onClick={() => setIsOpen(false)} aria-label="Close chat">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M18.3 5.70997C17.91 5.31997 17.28 5.31997 16.89 5.70997L12 10.59L7.10997 5.69997C6.71997 5.30997 6.08997 5.30997 5.69997 5.69997C5.30997 6.08997 5.30997 6.71997 5.69997 7.10997L10.59 12L5.69997 16.89C5.30997 17.28 5.30997 17.91 5.69997 18.3C6.08997 18.69 6.71997 18.69 7.10997 18.3L12 13.41L16.89 18.3C17.28 18.69 17.91 18.69 18.3 18.3C18.69 17.91 18.69 17.28 18.3 16.89L13.41 12L18.3 7.10997C18.68 6.72997 18.68 6.08997 18.3 5.70997Z" fill="white"/>
              </svg>
            </button>
          </div>

          <div className="message-list">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.role === 'user' ? 'message-user' : 'message-assistant'}`}
              >
                <div>{message.content}</div>
              </div>
            ))}
            {isLoading && (
              <div className="message message-assistant">
                <div className="loading">Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="input-box">
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Type your message..."
              disabled={isLoading}
              aria-label="Type your message"
            />
            <button
              onClick={handleSend}
              disabled={isLoading || !input.trim()}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatBot;