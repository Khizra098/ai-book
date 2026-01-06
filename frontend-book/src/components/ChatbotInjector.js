import React, { useEffect } from 'react';

// This component will render the chatbot UI
function ChatbotInjector() {
  useEffect(() => {
    const loadChatbot = async () => {
      try {
        // Wait for DOM to be ready
        if (document.readyState === 'loading') {
          await new Promise(resolve => {
            document.addEventListener('DOMContentLoaded', resolve);
          });
        }

        // Check if the container already exists
        let container = document.getElementById('rag-chatbot-container');
        if (!container) {
          // Create container element
          container = document.createElement('div');
          container.id = 'rag-chatbot-container';
          container.style.position = 'fixed';
          container.style.bottom = '20px';
          container.style.right = '20px';
          container.style.zIndex = '1000';
          container.style.width = '400px';
          container.style.height = '500px';
          document.body.appendChild(container);
        }

        // Dynamically import React and ReactDOM
        const [React, ReactDOMClient] = await Promise.all([
          import('react'),
          import('react-dom/client')
        ]);

        // Import the RAGChatbot component
        const { default: RAGChatbot } = await import('./RAGChatbot');

        // Create a wrapper component for the chatbot
        const ChatbotWrapper = () => (
          <RAGChatbot
            bookId={window.location.pathname.replace(/\//g, '_') || 'default_book'}
            apiEndpoint={window.RAG_CHATBOT_API_ENDPOINT || 'http://localhost:8000'}
          />
        );

        // Render the chatbot component
        const root = ReactDOMClient.createRoot(container);
        root.render(<ChatbotWrapper />);
      } catch (error) {
        console.error('Error loading chatbot:', error);
      }
    };

    loadChatbot();
  }, []);

  return null; // This component doesn't render anything itself
}

export default ChatbotInjector;