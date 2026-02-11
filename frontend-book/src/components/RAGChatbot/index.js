// import React, { useState, useRef, useEffect } from 'react';
// import './styles.css';

// const RAGChatbot = ({ bookId = 'default-book', apiEndpoint: propApiEndpoint }) => {
//   const apiEndpoint = propApiEndpoint || (typeof window !== 'undefined' && window.RAG_CHATBOT_API_ENDPOINT) || process.env.RAG_CHATBOT_API_ENDPOINT || process.env.REACT_APP_API_ENDPOINT || 'http://localhost:8000';
//   const [messages, setMessages] = useState([
//     { id: 1, text: "Hello! I'm your AI assistant. How can I help you with this book?", sender: 'bot', timestamp: new Date().toISOString() }
//   ]);
//   const [inputMessage, setInputMessage] = useState('');
//   const [isLoading, setIsLoading] = useState(false);
//   const [isOpen, setIsOpen] = useState(false); // Chatbot is closed by default
//   const messagesEndRef = useRef(null);
//   const textareaRef = useRef(null);

//   // No session initialization needed with the new API

//   // Auto-scroll to bottom when messages change
//   useEffect(() => {
//     scrollToBottom();
//   }, [messages]);

//   const scrollToBottom = () => {
//     messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
//   };

//   const handleSendMessage = async () => {
//     if (!inputMessage.trim() || isLoading) return;

//     const userMessage = {
//       id: Date.now(),
//       text: inputMessage,
//       sender: 'user',
//       timestamp: new Date().toISOString(),
//     };

//     setMessages((prev) => [...prev, userMessage]);
//     setInputMessage('');
//     setIsLoading(true);

//     try {
//       // Use the new API endpoint structure with timeout and better error handling
//       const controller = new AbortController();
//       const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

//       // Normalize the API endpoint to avoid duplicated slashes or '/api' segments
//       const normalizeApiBase = (endpoint) => {
//         if (!endpoint) return '';
//         // Remove trailing slashes
//         let base = endpoint.replace(/\/+$|\s+$/g, '');
//         // If someone provides a base that already includes '/api', strip it to avoid '/api/api' errors
//         base = base.replace(/\/api\s*$/i, '');
//         return base;
//       };

//       const baseUrl = normalizeApiBase(apiEndpoint);
//       const url = `${baseUrl}/api/query`;
//       console.debug('RAGChatbot: sending query to', url);

//       const response = await fetch(url, {
//         method: 'POST',
//         headers: {
//           'Content-Type': 'application/json',
//         },
//         body: JSON.stringify({
//           query: inputMessage,
//           parameters: {
//             temperature: 0.7,
//             max_tokens: 1000,
//             top_p: 0.9
//           }
//         }),
//         signal: controller.signal
//       });

//       clearTimeout(timeoutId);

//       if (response.ok) {
//         const data = await response.json();
//         const botMessage = {
//           id: Date.now() + 1,
//           text: data.response,
//           sender: 'bot',
//           timestamp: data.timestamp,
//           sources: data.sources,
//         };
//         setMessages((prev) => [...prev, botMessage]);
//       } else {
//         // Provide a clearer message for common misconfiguration cases (404)
//         if (response.status === 404) {
//           const errorMessage = {
//             id: Date.now() + 1,
//             text: `Error: Server returned 404 Not Found for ${url}. Please verify the backend is running and that the configured API endpoint (${apiEndpoint}) is correct.`,
//             sender: 'bot',
//             timestamp: new Date().toISOString(),
//           };
//           setMessages((prev) => [...prev, errorMessage]);
//         } else {
//           const contentType = response.headers.get('content-type');
//           let errorData;

//           if (contentType && contentType.includes('application/json')) {
//             errorData = await response.json();
//           } else {
//             errorData = { error: await response.text() || 'Failed to get response' };
//           }

//           const errorMessage = {
//             id: Date.now() + 1,
//             text: `Error: ${errorData.error || errorData.detail || `Server returned ${response.status} ${response.statusText}`}`,
//             sender: 'bot',
//             timestamp: new Date().toISOString(),
//           };
//           setMessages((prev) => [...prev, errorMessage]);
//         }
//       }
//     } catch (error) {
//       if (error.name === 'AbortError') {
//         const errorMessage = {
//           id: Date.now() + 1,
//           text: 'Error: Request timed out. The server may be unavailable or processing your request is taking too long.',
//           sender: 'bot',
//           timestamp: new Date().toISOString(),
//         };
//         setMessages((prev) => [...prev, errorMessage]);
//       } else if (error.name === 'TypeError' && error.message.includes('fetch')) {
//         const errorMessage = {
//           id: Date.now() + 1,
//           text: 'Error: Failed to connect to the server. Please make sure the backend service is running and accessible. Check if the backend is running on the configured API endpoint.',
//           sender: 'bot',
//           timestamp: new Date().toISOString(),
//         };
//         setMessages((prev) => [...prev, errorMessage]);
//       } else {
//         const errorMessage = {
//           id: Date.now() + 1,
//           text: `Error: ${error.message || 'Network error'}`,
//           sender: 'bot',
//           timestamp: new Date().toISOString(),
//         };
//         setMessages((prev) => [...prev, errorMessage]);
//       }
//     } finally {
//       setIsLoading(false);
//     }
//   };

//   const handleKeyDown = (e) => {
//     if (e.key === 'Enter' && !e.shiftKey) {
//       e.preventDefault();
//       handleSendMessage();
//     }
//   };

//   const toggleChatbot = () => {
//     setIsOpen(!isOpen);
//   };

//   // When closed, show only the toggle button
//   if (!isOpen) {
//     return (
//       <button
//         className="chatbot-toggle-button"
//         onClick={toggleChatbot}
//         aria-label="Open chatbot"
//       >
//         💬
//       </button>
//     );
//   }

//   // When open, show the full chatbot
//   return (
//     <div className="rag-chatbot-container">
//       <div className="rag-chatbot-header">
//         <h3>AI Assistant</h3>
//         <button
//           className="chatbot-close-button"
//           onClick={toggleChatbot}
//           aria-label="Close chatbot"
//         >
//           ×
//         </button>
//       </div>

//       <div className="rag-chatbot-messages">
//         {messages.map((message) => (
//           <div
//             key={message.id}
//             className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
//           >
//             <div className="message-content">
//               <strong>{message.sender === 'user' ? 'You:' : 'Assistant:'}</strong>
//               <p>{message.text}</p>
//               {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
//                 <details className="retrieved-context">
//                   <summary>Sources</summary>
//                   {message.sources.map((source, index) => (
//                     <div key={index} className="source-chunk">
//                       <a href={source.url} target="_blank" rel="noopener noreferrer">
//                         {source.title}
//                       </a>
//                       <p>{source.content.substring(0, 100)}{source.content.length > 100 ? '...' : ''}</p>
//                     </div>
//                   ))}
//                 </details>
//               )}
//             </div>
//           </div>
//         ))}
//         {isLoading && (
//           <div className="message bot-message">
//             <div className="message-content">
//               <strong>Assistant:</strong>
//               <div className="typing-indicator">
//                 <span></span>
//                 <span></span>
//                 <span></span>
//               </div>
//             </div>
//           </div>
//         )}
//         <div ref={messagesEndRef} />
//       </div>

//       <div className="rag-chatbot-input">
//         <textarea
//           ref={textareaRef}
//           value={inputMessage}
//           onChange={(e) => setInputMessage(e.target.value)}
//           onKeyDown={handleKeyDown}
//           placeholder="Ask a question about this book..."
//           rows="2"
//           disabled={isLoading}
//         />
//         <button
//           onClick={handleSendMessage}
//           disabled={!inputMessage.trim() || isLoading}
//           className="send-button"
//         >
//           {isLoading ? 'Sending...' : 'Send'}
//         </button>
//       </div>
//     </div>
//   );
// };





import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

const RAGChatbot = ({ bookId = 'default-book' }) => {
  const apiEndpoint = 'https://khizra098-book-backend.hf.space';


  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your AI assistant. How can I help you with this book?", sender: 'bot', timestamp: new Date().toISOString() }
  ]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false); // Chatbot is closed by default
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

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
      // Timeout controller
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 sec timeout

      // Normalize API base to avoid double slashes or /api/api
      const normalizeApiBase = (endpoint) => {
        let base = endpoint.replace(/\/+$/, ''); // remove trailing slash
        base = base.replace(/\/api\s*$/i, '');  // remove trailing /api
        return base;
      };

      const baseUrl = normalizeApiBase(apiEndpoint);
      const url = `${baseUrl}/api/query`;

      const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: inputMessage,
          parameters: { temperature: 0.7, max_tokens: 1000, top_p: 0.9 }
        }),
        signal: controller.signal
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          timestamp: data.timestamp,
          sources: data.sources,
        };
        setMessages((prev) => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: Server returned ${response.status} ${response.statusText}. Check backend URL: ${apiEndpoint}`,
          sender: 'bot',
          timestamp: new Date().toISOString(),
        };
        setMessages((prev) => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: error.name === 'AbortError'
          ? 'Error: Request timed out. Server may be unavailable.'
          : 'Error: Failed to connect to the server. Make sure the backend is running.',
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

  const toggleChatbot = () => setIsOpen(!isOpen);

  if (!isOpen) {
    return (
      <button className="chatbot-toggle-button" onClick={toggleChatbot} aria-label="Open chatbot">
        💬
      </button>
    );
  }

  return (
    <div className="rag-chatbot-container">
      <div className="rag-chatbot-header">
        <h3>AI Assistant</h3>
        <button className="chatbot-close-button" onClick={toggleChatbot} aria-label="Close chatbot">×</button>
      </div>

      <div className="rag-chatbot-messages">
        {messages.map((message) => (
          <div key={message.id} className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}>
            <div className="message-content">
              <strong>{message.sender === 'user' ? 'You:' : 'Assistant:'}</strong>
              <p>{message.text}</p>
              {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                <details className="retrieved-context">
                  <summary>Sources</summary>
                  {message.sources.map((source, index) => (
                    <div key={index} className="source-chunk">
                      <a href={source.url} target="_blank" rel="noopener noreferrer">{source.title}</a>
                      <p>{source.content.substring(0, 100)}{source.content.length > 100 ? '...' : ''}</p>
                    </div>
                  ))}
                </details>
              )}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <strong>Assistant:</strong>
              <div className="typing-indicator"><span></span><span></span><span></span></div>
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
          disabled={isLoading}
        />
        <button onClick={handleSendMessage} disabled={!inputMessage.trim() || isLoading} className="send-button">
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default RAGChatbot;

// export default RAGChatbot;
