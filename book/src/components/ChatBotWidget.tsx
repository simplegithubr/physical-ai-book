import React, { useState, useRef, useEffect } from 'react';
import './ChatBotWidget.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

const ChatBotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(true);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Add welcome message when chat opens
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([
        {
          id: 'welcome',
          role: 'assistant',
          content: "Hello! I'm your book content assistant. You can ask me questions about the Physical AI and Humanoid Robotics content.",
          timestamp: new Date()
        }
      ]);
    }
  }, [isOpen, messages.length]);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages, isLoading]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date()
    };

    // Add user message to UI immediately
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Make API call to backend
      // const response = await fetch("https://sagardeveloper-rag_chatbord.hf.space/widget", {
      //   method: 'POST',
      //   headers: {
      //     'Content-Type': 'application/json',
      //   },
      //   body: JSON.stringify({
      //     query: userMessage.content,
      //     book_id: 'physical-ai'
      //   })
      // });
//       const response = await fetch("https://sagardeveloper-rag_chatbord.hf.space/chat", {
//   method: 'POST',
//   headers: { 'Content-Type': 'application/json' },
//   body: JSON.stringify({
//     query: userMessage.content,
//     selected_text: "",
//     history: [],
//     book_id: "physical-ai",
//     temperature: 0.7
//   })
// });
// const response = await fetch("https://sagardeveloper-rag_chatbord.hf.space/widget", {
//   method: 'POST',
//   headers: { 'Content-Type': 'application/json' },
//   body: JSON.stringify({
//     query: userMessage.content,
//     selected_text: "",
//     history: [],
//     book_id: "physical-ai",
//     temperature: 0.7
//   })
// });
const response = await fetch("https://sagardeveloper-rag-chatbord.hf.space/chat", {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    query: userMessage.content,
    selected_text: "",
    history: [],  // ya messages ka history bhej sakte ho for better context
    book_id: "physical-ai",
    temperature: 0.7
  })
});
      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      // const data = await response.json();

      // const assistantMessage: Message = {
      //   id: `assistant-${Date.now()}`,
      //   role: 'assistant',
      //   content: data.response,
      //   timestamp: new Date()
      // };
      const data = await response.json();

const assistantMessage: Message = {
  id: `assistant-${Date.now()}`,
  role: 'assistant',
  // content: data.response || "No response",  // yeh line confirm kar
  content: data.response || data.answer || "No response received",
  timestamp: new Date()
};

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: "Sorry, I encountered an error processing your request. Please try again.",
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <div className="chatbot-header-content">
              <h3>Physical AI Assistant</h3>
              <div className={`online-status ${isOnline ? 'online' : 'offline'}`}>
                <span className="status-indicator"></span>
                <span className="status-text">{isOnline ? '● Online' : '● Offline'}</span>
              </div>
              <button className="close-button" onClick={closeChat}>
                ×
              </button>
            </div>
          </div>
          <div className="chatbot-body">
            <div className="messages-container">
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                  style={{
                    alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                    backgroundColor: message.role === 'user' ? '#156e3aff' : '#f3f4f6',
                    color: message.role === 'user' ? 'white' : 'black'
                  }}
                >
                  {message.content}
                </div>
              ))}
              {isLoading && (
                <div
                  className="message assistant"
                  style={{
                    alignSelf: 'flex-start',
                    backgroundColor: '#f3f4f6',
                    color: 'black'
                  }}
                >
                  <span>Thinking</span>
                  <span className="loading-dots">
                    <span className="loading-dot">.</span>
                    <span className="loading-dot">.</span>
                    <span className="loading-dot">.</span>
                  </span>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className="input-container">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question about the book..."
                className="user-input"
                rows={1}
                disabled={isLoading}
              />
              <button
                onClick={handleSendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-button"
              >
                Send
              </button>
            </div>
          </div>
        </div>
      ) : null}

      <button className="chatbot-toggle" onClick={toggleChat}>
        <span className="chatbot-icon">🤖</span>
      </button>
    </div>
  );
};

export default ChatBotWidget;