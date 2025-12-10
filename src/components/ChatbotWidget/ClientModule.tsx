import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatbotWidget from './ChatbotWidget';

const ClientModule: React.FC = () => {
  const location = useLocation();

  useEffect(() => {
    // This effect runs on every route change
    // You can add any logic that needs to run when the page changes
  }, [location]);

  return <ChatbotWidget />;
};

export default ClientModule;