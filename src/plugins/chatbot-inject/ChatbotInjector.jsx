import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';

// This component gets injected into all pages
export default function ChatbotInjector() {
  const location = useLocation();

  // Only show on certain routes if needed, or show on all
  return <ChatbotWidget />;
}
// import React from "react";
// import ChatbotWidget from "@site/src/components/ChatbotWidget/ChatbotWidget";

// export default function Root({ children }) {
//   return (
//     <>
//       {children}
//       <ChatbotWidget />
//     </>
//   );
// }
