import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

const RAGChatbot = ({ bookId = 'default-book', apiEndpoint = 'http://localhost:8000/api/v1' }) => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [contextMode, setContextMode] = useState('global'); // 'global' or 'selection'
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Initialize session when component mounts
  useEffect(() => {
    const initSession = async () => {
      try {
        const response = await fetch(`${apiEndpoint}/chat/sessions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ book_id: bookId }),
        });

        if (response.ok) {
          const data = await response.json();
          setSessionId(data.session_id);
        } else {
          console.error('Failed to create session');
        }
      } catch (error) {
        console.error('Error creating session:', error);
      }
    };

    initSession();
  }, [bookId, apiEndpoint]);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async () => {
    if (!inputMessage.trim() || !sessionId || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputMessage,
      sender: 'user',
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);

    try {
      const response = await fetch(`${apiEndpoint}/chat/sessions/${sessionId}/messages`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputMessage,
          selected_text: contextMode === 'selection' ? selectedText : null,
          context_mode: contextMode,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          timestamp: data.timestamp,
          retrievedChunks: data.retrieved_chunks,
        };
        setMessages((prev) => [...prev, botMessage]);
      } else {
        const errorData = await response.json();
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${errorData.detail || 'Failed to get response'}`,
          sender: 'bot',
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${error.message || 'Network error'}`,
        sender: 'bot',
        timestamp: new Date().toISOString(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleTextSelection = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
      setContextMode('selection');
      // Optionally pre-fill the input with a question about the selection
      if (!inputMessage) {
        setInputMessage(`Explain this: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
      }
    }
  };

  const clearSelection = () => {
    setSelectedText('');
    setContextMode('global');
  };

  return (
    <div className="rag-chatbot-container">
      <div className="rag-chatbot-header">
        <h3>Book Assistant</h3>
        <div className="context-mode-selector">
          <button
            className={`mode-btn ${contextMode === 'global' ? 'active' : ''}`}
            onClick={() => setContextMode('global')}
            title="Search entire book"
          >
            Global
          </button>
          <button
            className={`mode-btn ${contextMode === 'selection' ? 'active' : ''}`}
            onClick={handleTextSelection}
            title="Use selected text only"
          >
            Selection
          </button>
        </div>
        {selectedText && (
          <div className="selected-text-preview">
            <small>Context: "{selectedText.substring(0, 60)}..."</small>
            <button onClick={clearSelection} className="clear-selection-btn">×</button>
          </div>
        )}
      </div>

      <div className="rag-chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your book assistant. Ask me questions about this book, and I'll answer based on the content.</p>
            <p>You can:</p>
            <ul>
              <li>Ask general questions about the book</li>
              <li>Select text and ask questions about it</li>
              <li>Switch between global and selection modes</li>
            </ul>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                <strong>{message.sender === 'user' ? 'You:' : 'Assistant:'}</strong>
                <p>{message.text}</p>
                {message.sender === 'bot' && message.retrievedChunks && message.retrievedChunks.length > 0 && (
                  <details className="retrieved-context">
                    <summary>Referenced content</summary>
                    {message.retrievedChunks.slice(0, 2).map((chunk, index) => (
                      <div key={index} className="retrieved-chunk">
                        <p>{chunk.text}</p>
                      </div>
                    ))}
                  </details>
                )}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <strong>Assistant:</strong>
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="rag-chatbot-input">
        <textarea
          ref={textareaRef}
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about this book..."
          rows="2"
          disabled={isLoading || !sessionId}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputMessage.trim() || !sessionId || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default RAGChatbot;