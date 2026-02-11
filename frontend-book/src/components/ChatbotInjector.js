// import React, { useEffect } from 'react';

// // This component will render the chatbot UI
// function ChatbotInjector() {
//   useEffect(() => {
//     const loadChatbot = async () => {
//       try {
//         // Wait for DOM to be ready
//         if (document.readyState === 'loading') {
//           await new Promise(resolve => {
//             document.addEventListener('DOMContentLoaded', resolve);
//           });
//         }

//         // Check if the container already exists
//         let container = document.getElementById('rag-chatbot-container');
//         if (!container) {
//           // Create container element
//           container = document.createElement('div');
//           container.id = 'rag-chatbot-container';
//           container.style.position = 'fixed';
//           container.style.bottom = '20px';
//           container.style.right = '20px';
//           container.style.zIndex = '1000';
//           container.style.width = '400px';
//           container.style.height = '500px';
//           document.body.appendChild(container);
//         }

//         // Dynamically import React and ReactDOM
//         const [React, ReactDOMClient] = await Promise.all([
//           import('react'),
//           import('react-dom/client')
//         ]);

//         // Import the RAGChatbot component
//         const { default: RAGChatbot } = await import('./RAGChatbot');

//         // Create a wrapper component for the chatbot
//         const ChatbotWrapper = () => (
//           <RAGChatbot
//             bookId={window.location.pathname.replace(/\//g, '_') || 'default_book'}
//             apiEndpoint={window.RAG_CHATBOT_API_ENDPOINT || 'http://localhost:8000'}
//           />
//         );

//         // Render the chatbot component
//         const root = ReactDOMClient.createRoot(container);
//         root.render(<ChatbotWrapper />);
//       } catch (error) {
//         console.error('Error loading chatbot:', error);
//       }
//     };

//     loadChatbot();
//   }, []);

//   return null; // This component doesn't render anything itself
// }

// export default ChatbotInjector;








import React, { useState } from 'react';

function RAGChatbot({ bookId, apiEndpoint }) {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  // Ensure no trailing slash at the end of endpoint
  // const endpoint = (apiEndpoint || 'https://khizra098-book-backend.hf.space').replace(/\/+$/, '');
  const endpoint = 'https://khizra098-book-backend.hf.space';


  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = input;
    setMessages(prev => [...prev, { sender: 'user', text: userMessage }]);
    setInput('');

    try {
      const response = await fetch(`${endpoint}/api/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: userMessage })
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = data.response || 'No response';
      setMessages(prev => [...prev, { sender: 'bot', text: botMessage }]);

    } catch (error) {
      console.error('Error sending message:', error);
      setMessages(prev => [...prev, { sender: 'bot', text: 'Failed to connect to server' }]);
    }
  };

  const handleKeyPress = e => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100%',
      border: '1px solid #ccc',
      borderRadius: '8px',
      background: '#fff',
      padding: '10px'
    }}>
      <div style={{ flex: 1, overflowY: 'auto', marginBottom: '10px' }}>
        {messages.map((msg, idx) => (
          <div key={idx} style={{ margin: '5px 0', textAlign: msg.sender === 'user' ? 'right' : 'left' }}>
            <span style={{
              display: 'inline-block',
              padding: '6px 10px',
              borderRadius: '12px',
              background: msg.sender === 'user' ? '#007bff' : '#f1f0f0',
              color: msg.sender === 'user' ? '#fff' : '#000'
            }}>{msg.text}</span>
          </div>
        ))}
      </div>
      <input
        type="text"
        value={input}
        onChange={e => setInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Type your message..."
        style={{ padding: '8px', borderRadius: '8px', border: '1px solid #ccc' }}
      />
      <button onClick={sendMessage} style={{ marginTop: '5px', padding: '8px', borderRadius: '8px', cursor: 'pointer' }}>
        Send
      </button>
    </div>
  );
}

export default RAGChatbot;
