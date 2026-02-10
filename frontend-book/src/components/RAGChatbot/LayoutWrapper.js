import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

// Main layout wrapper that ensures chatbot is always present
export default function LayoutWrapper({ children, ...props }) {
  const location = useLocation();

  useEffect(() => {
    // Dynamically import and render the chatbot component
    const loadChatbot = async () => {
      try {
        // Import React and ReactDOM for client-side rendering
        const [{ default: React }, { createRoot }] = await Promise.all([
          import('react'),
          import('react-dom/client')
        ]);

        // Import the chatbot component
        const { default: RAGChatbot } = await import('./index');

        // Check if container already exists to avoid duplicates
        let container = document.getElementById('rag-chatbot-container');
        if (!container) {
          // Create container if it doesn't exist
          container = document.createElement('div');
          container.id = 'rag-chatbot-container';
          container.style.position = 'fixed';
          container.style.bottom = '20px';
          container.style.right = '20px';
          container.style.zIndex = '1000';
          document.body.appendChild(container);
        }

        // Create the chatbot wrapper component
        
       const RAGChatbotWrapper = () => (
        <RAGChatbot
        bookId={location.pathname.replace(/\//g, '_') || 'default_book'}
        apiEndpoint={(process.env.RAG_CHATBOT_API_ENDPOINT || 'https://khizra098-book-backend.hf.space').replace(/\/+$/, '')}
       />
       );


        // Render the chatbot
        const root = createRoot(container);
        root.render(<RAGChatbotWrapper />);
      } catch (error) {
        console.error('Failed to load RAG Chatbot:', error);
      }
    };

    // Only load after DOM is ready
    if (typeof window !== 'undefined' && document.readyState !== 'loading') {
      loadChatbot();
    } else if (typeof window !== 'undefined') {
      window.addEventListener('DOMContentLoaded', loadChatbot);
    }
  }, [location.pathname]);

  return <>{children}</>;
}
