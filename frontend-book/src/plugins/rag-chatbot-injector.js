import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import React, { useEffect } from 'react';

if (ExecutionEnvironment.canUseDOM) {
  // Dynamically import the RAG Chatbot component
  import('../components/RAGChatbot').then(({ default: RAGChatbot }) => {
    const container = document.getElementById('rag-chatbot-container');
    if (container) {
      // Get the current book/page ID from the URL or metadata
      const bookId = window.location.pathname.replace(/\//g, '_') || 'default_book';

      // Render the chatbot component
      const RAGChatbotWrapper = () => (
        <RAGChatbot
          bookId={bookId}
          apiEndpoint={process.env.RAG_CHATBOT_API_ENDPOINT || '/api/v1'}
        />
      );

      // Use React to render the component
      const React = require('react');
      const ReactDOM = require('react-dom/client');

      const root = ReactDOM.createRoot(container);
      root.render(<RAGChatbotWrapper />);
    }
  }).catch(err => {
    console.error('Failed to load RAG Chatbot component:', err);
  });
}

// This module doesn't export anything but runs the side effect above