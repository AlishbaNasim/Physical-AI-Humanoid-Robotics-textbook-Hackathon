import React from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';

function ChatbotPage(): JSX.Element {
  return (
    <Layout
      title="Book Assistant"
      description="AI-powered assistant for the Physical AI & Humanoid Robotics book">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <div className="text--center padding-horiz--md">
              <h1>Book Assistant</h1>
              <p className="hero__subtitle">
                Ask questions about the Physical AI & Humanoid Robotics book content
              </p>
            </div>

            <div style={{
              display: 'flex',
              justifyContent: 'center',
              marginTop: '2rem',
              maxWidth: '800px',
              margin: '2rem auto 0'
            }}>
              <div style={{ width: '100%', maxWidth: '600px' }}>
                <ChatbotWidget />
              </div>
            </div>

            <div className="text--center padding-vert--lg">
              <p>
                Select text on any page and click "Ask about this text" to get specific answers
                based on the book content with proper source attribution.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ChatbotPage;