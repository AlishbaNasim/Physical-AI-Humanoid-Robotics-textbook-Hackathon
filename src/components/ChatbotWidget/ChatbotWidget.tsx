import React, { useState, useEffect, useRef } from 'react';
import './ChatbotWidget.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{
    content_id: string;
    title: string;
    url: string;
    text: string;
    relevance_score: number;
  }>;
}

const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim() || '';
      setSelectedText(selectedText);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          selected_text: selectedText, // Include selected text if available
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Create assistant message from API response
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date(),
        sources: data.sources || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendSelectedText = () => {
    if (selectedText) {
      setInputValue(selectedText);
    }
  };

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>Book Assistant</h3>
            <button className="chatbot-close" onClick={toggleChat}>
              ×
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>Hello! I'm your Physical AI & Humanoid Robotics book assistant.</p>
                <p>Ask me questions about the content, or select text on the page and I can provide more information.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chatbot-message ${message.role}`}
                >
                  <div className="message-content">
                    {message.content}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <details>
                        <summary>Sources:</summary>
                        <ul>
                          {message.sources.map((source, index) => (
                            <li key={index}>
                              <a href={source.url} target="_blank" rel="noopener noreferrer">
                                {source.title}
                              </a>
                              <p className="source-text">{source.text.substring(0, 100)}...</p>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                  )}
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className="selected-text-prompt">
              <p>Selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</p>
              <button onClick={sendSelectedText} className="use-selected-btn">
                Ask about this text
              </button>
            </div>
          )}

          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about the book content..."
              disabled={isLoading}
              className="chatbot-input"
            />
            <button type="submit" disabled={isLoading} className="chatbot-send-btn">
              {isLoading ? 'Sending...' : '→'}
            </button>
          </form>
        </div>
      ) : (
        <button className="chatbot-toggle" onClick={toggleChat}>
          💬
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;