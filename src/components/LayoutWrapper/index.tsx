import React from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from '../ChatbotWidget/ChatbotWidget';

interface LayoutWrapperProps {
  children: React.ReactNode;
  [key: string]: any;
}

const LayoutWrapper: React.FC<LayoutWrapperProps> = ({ children, ...layoutProps }) => {
  return (
    <Layout {...layoutProps}>
      {children}
      <ChatbotWidget />
    </Layout>
  );
};

export default LayoutWrapper;