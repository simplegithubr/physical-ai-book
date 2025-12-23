// /* RAG Chatbot Widget JavaScript */

class ChatWidget {
  constructor() {
    this.chatContainer = document.getElementById('chat-container');
    this.messagesContainer = document.getElementById('chat-messages');
    this.userInput = document.getElementById('user-input');
    this.sendButton = document.getElementById('send-button');
    this.apiBaseUrl = this.getApiBaseUrl(); // Dynamically determine API base URL

    this.initializeEventListeners();
    this.addWelcomeMessage();
  }

  getApiBaseUrl() {
    // Try to get API base URL from data attribute, otherwise use current origin
    const container = document.getElementById('chat-container');
    if (container && container.dataset.api_base) {
      return container.dataset.api_base;
    }
    // Default to the same origin as the page
    return `${window.location.protocol}//${window.location.host}`;
  }

  initializeEventListeners() {
    this.sendButton.addEventListener('click', () => this.sendMessage());
    this.userInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });

    // Listen for selected text from parent page (Task T48: Implement postMessage API for selected text communication)
    window.addEventListener('message', (event) => {
      // Verify the origin if needed for security
      // if (event.origin !== window.location.origin) return;

      if (event.data && event.data.type === 'TEXT_SELECTED') {
        this.handleSelectedText(event.data.text);
      }
    });
  }

  addWelcomeMessage() {
    const welcomeMessage = this.createMessageElement(
      'assistant',
      "Hello! I'm your book content assistant. You can ask me questions about the book content, or select text in the book and I'll answer questions specifically about that text."
    );
    this.messagesContainer.appendChild(welcomeMessage);
    this.scrollToBottom();
  }

  async sendMessage() {
    const query = this.userInput.value.trim();
    if (!query) return;

    // Disable input while processing
    this.userInput.disabled = true;
    this.sendButton.disabled = true;

    try {
      // Add user message to UI
      const userMessage = this.createMessageElement('user', query);
      this.messagesContainer.appendChild(userMessage);
      this.scrollToBottom();

      // Clear input
      this.userInput.value = '';

      // Show loading indicator
      const loadingElement = this.createLoadingElement();
      this.messagesContainer.appendChild(loadingElement);
      this.scrollToBottom();

      // Prepare request
      const requestBody = {
        query: query,
        book_id: "physical-ai"
      };

      // Include selected text if available
      if (this.selectedText) {
        requestBody.selected_text = this.selectedText;
      }

      // Make API call
      const response = await fetch(`${this.apiBaseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      let data;
      try {
        data = await response.json();
      } catch (parseError) {
        // Remove loading indicator
        try {
          this.messagesContainer.removeChild(loadingElement);
        } catch (removeError) {
          // Ignore error if element was already removed
        }
        throw new Error(`Invalid response format: ${parseError.message}`);
      }

      // Remove loading indicator
      try {
        this.messagesContainer.removeChild(loadingElement);
      } catch (removeError) {
        // Ignore error if element was already removed
      }

      // Add assistant response
      const assistantMessage = this.createMessageElement('assistant', data.response);
      this.messagesContainer.appendChild(assistantMessage);

      // Add sources if available
      if (data.sources && data.sources.length > 0) {
        const sourcesMessage = this.createSourcesElement(data.sources);
        this.messagesContainer.appendChild(sourcesMessage);
      }

      this.scrollToBottom();

      // Clear selected text after use
      this.selectedText = null;

    } catch (error) {
      console.error('Error sending message:', error);

      // Remove loading indicator
      const loadingElements = this.messagesContainer.querySelectorAll('.message.loading');
      loadingElements.forEach(el => el.remove());

      // Show error message
      const errorMessage = this.createMessageElement('assistant',
        `Sorry, I encountered an error processing your request: ${error.message}. Please try again.`);
      this.messagesContainer.appendChild(errorMessage);
      this.scrollToBottom();
    } finally {
      // Re-enable input
      this.userInput.disabled = false;
      this.sendButton.disabled = false;
    }
  }

  handleSelectedText(text) {
    if (!text || text.length === 0) return;

    this.selectedText = text;

    // Show feedback to user
    const feedbackMessage = this.createMessageElement(
      'assistant',
      `I've received the selected text: "${text.substring(0, 50)}${text.length > 50 ? '...' : ''}". You can now ask questions specifically about this text.`
    );
    this.messagesContainer.appendChild(feedbackMessage);
    this.scrollToBottom();
  }

  createMessageElement(role, content) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${role}`;
    messageDiv.textContent = content;
    return messageDiv;
  }

  createSourcesElement(sources) {
    const sourcesDiv = document.createElement('div');
    sourcesDiv.className = 'message sources';

    const header = document.createElement('strong');
    header.textContent = 'Sources:';
    sourcesDiv.appendChild(header);

    const sourcesList = document.createElement('ul');
    sources.slice(0, 3).forEach(source => { // Show max 3 sources
      const listItem = document.createElement('li');

      const link = document.createElement('a');
      link.href = source.url;
      link.textContent = source.section || 'Reference';
      link.target = '_blank';
      link.rel = 'noopener noreferrer';

      listItem.appendChild(link);
      sourcesList.appendChild(listItem);
    });

    sourcesDiv.appendChild(sourcesList);
    return sourcesDiv;
  }

  createLoadingElement() {
    const loadingDiv = document.createElement('div');
    loadingDiv.className = 'message assistant loading';

    const textSpan = document.createElement('span');
    textSpan.textContent = 'Thinking';

    const dotsSpan = document.createElement('span');
    dotsSpan.className = 'loading-dots';

    for (let i = 0; i < 3; i++) {
      const dot = document.createElement('span');
      dot.className = 'loading-dot';
      dotsSpan.appendChild(dot);
    }

    loadingDiv.appendChild(textSpan);
    loadingDiv.appendChild(dotsSpan);

    return loadingDiv;
  }

  scrollToBottom() {
    // Scroll to the bottom to show the latest message
    // Using scrollIntoView for more reliable scrolling
    if (this.messagesContainer.lastChild) {
      this.messagesContainer.lastChild.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    }
  }

}

// Initialize the chat widget when the DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  new ChatWidget();

  // Ensure proper height when in iframe
  if (window.self !== window.top) {
    // We're in an iframe - ensure the container takes full height
    const container = document.getElementById('chat-container');
    if (container) {
      container.style.height = '100%';
      container.style.minHeight = '300px'; // Minimum height for usability
    }
  }
});

// Optional: Function to send selected text from parent page
function sendSelectedTextToWidget(text) {
  // Send message to the widget iframe
  const iframe = document.querySelector('#chat-container iframe'); // if embedded in iframe
  if (iframe) {
    iframe.contentWindow.postMessage({
      type: 'TEXT_SELECTED',
      text: text
    }, '*');
  } else {
    // If not in iframe, send to parent
    window.parent.postMessage({
      type: 'TEXT_SELECTED',
      text: text
    }, '*');
  }
}

// Optional: Function to select text and send it to the widget
function setupTextSelectionListener() {
  let selectionTimer;

  document.addEventListener('mouseup', () => {
    clearTimeout(selectionTimer);
    selectionTimer = setTimeout(() => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        // Send to chat widget
        window.postMessage({
          type: 'TEXT_SELECTED',
          text: selectedText
        }, '*');
      }
    }, 150); // Small delay to allow selection to complete
  });
}

// Set up text selection if needed
// setupTextSelectionListener();

