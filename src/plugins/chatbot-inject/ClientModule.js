/**
 * Docusaurus Client Module for Chatbot Widget
 * This module gets injected into all pages to provide the chatbot functionality
 */

import React from 'react';
import {createRoot} from 'react-dom/client';
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';

// Create a wrapper component that renders the chatbot widget
function ChatbotWrapper() {
  return <ChatbotWidget />;
}

export default ChatbotWrapper;