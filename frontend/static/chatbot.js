(function() {
  const API_URL = 'https://misbah-asghar-texbook-backend.hf.space/api/chat';

  const styles = `
    #chatbot-button {
      position: fixed;
      bottom: 28px;
      right: 28px;
      width: 60px;
      height: 60px;
      border-radius: 50%;
      background: linear-gradient(135deg, #4a90e2 0%, #5c6bc0 100%);
      border: none;
      cursor: pointer;
      box-shadow: 0 4px 16px rgba(74, 144, 226, 0.35);
      z-index: 9999;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 26px;
      color: white;
      transition: all 0.3s ease;
    }
    #chatbot-button:hover {
      transform: scale(1.08);
      box-shadow: 0 6px 20px rgba(74, 144, 226, 0.45);
    }
    #chatbot-panel {
      position: fixed;
      bottom: 100px;
      right: 28px;
      top: 80px;
      width: 420px;
      max-height: calc(100vh - 180px);
      background: var(--ifm-background-color);
      border-radius: 16px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.12);
      display: none;
      flex-direction: column;
      z-index: 9999;
      overflow: hidden;
      border: 1px solid rgba(0,0,0,0.08);
    }
    @media screen and (max-width: 768px) {
      #chatbot-panel {
        width: calc(100vw - 32px);
        right: 16px;
        left: 16px;
        bottom: 80px;
        top: 100px;
        max-height: calc(100vh - 180px);
      }
      #chatbot-button {
        bottom: 16px;
        right: 16px;
        width: 56px;
        height: 56px;
        font-size: 24px;
      }
    }
    @media screen and (max-width: 480px) {
      #chatbot-panel {
        top: 120px;
        bottom: 70px;
        max-height: calc(100vh - 190px);
      }
    }
    #chatbot-panel.open {
      display: flex;
      animation: slideUp 0.3s ease;
    }
    @keyframes slideUp {
      from { opacity: 0; transform: translateY(20px); }
      to { opacity: 1; transform: translateY(0); }
    }
    #chatbot-header {
      background: linear-gradient(135deg, #4a90e2 0%, #5c6bc0 100%);
      color: white;
      padding: 20px 24px;
      font-weight: 500;
      font-size: 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      letter-spacing: 0.3px;
    }
    #chatbot-close {
      background: rgba(255,255,255,0.15);
      border: none;
      color: white;
      font-size: 18px;
      cursor: pointer;
      width: 30px;
      height: 30px;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: background 0.2s;
    }
    #chatbot-close:hover {
      background: rgba(255,255,255,0.25);
    }
    #chatbot-messages {
      flex: 1;
      overflow-y: auto;
      padding: 24px;
      display: flex;
      flex-direction: column;
      gap: 20px;
      background: #fafbfc;
    }
    html[data-theme='dark'] #chatbot-messages {
      background: #1e1e1e;
    }
    .chat-message {
      padding: 14px 18px;
      border-radius: 14px;
      max-width: 82%;
      word-wrap: break-word;
      line-height: 1.6;
      animation: fadeIn 0.3s ease;
      font-size: 14.5px;
    }
    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }
    .chat-message.user {
      background: linear-gradient(135deg, #4a90e2 0%, #5c6bc0 100%);
      color: white;
      align-self: flex-end;
      box-shadow: 0 2px 6px rgba(74, 144, 226, 0.25);
    }
    .chat-message.bot {
      background: #ffffff;
      color: #2c3e50;
      align-self: flex-start;
      border: 1px solid #e1e8ed;
      box-shadow: 0 1px 3px rgba(0,0,0,0.08);
    }
    html[data-theme='dark'] .chat-message.bot {
      background: #2a2a2a;
      color: #e8e8e8;
      border: 1px solid #3a3a3a;
    }
    .chat-message.loading {
      background: #f0f2f5;
      color: #6c757d;
      align-self: flex-start;
      font-style: italic;
      opacity: 0.8;
      border: 1px solid #e1e8ed;
    }
    html[data-theme='dark'] .chat-message.loading {
      background: #2a2a2a;
      color: #9a9a9a;
      border: 1px solid #3a3a3a;
    }
    .chat-sources {
      font-size: 11.5px;
      color: #6c757d;
      margin-top: 10px;
      padding-top: 10px;
      border-top: 1px solid #e8ecef;
      font-weight: 500;
    }
    html[data-theme='dark'] .chat-sources {
      color: #9a9a9a;
      border-top: 1px solid #3a3a3a;
    }
    #chatbot-input-container {
      padding: 18px 20px;
      border-top: 1px solid #e1e8ed;
      display: flex;
      gap: 12px;
      background: #ffffff;
    }
    html[data-theme='dark'] #chatbot-input-container {
      background: #1e1e1e;
      border-top: 1px solid #3a3a3a;
    }
    #chatbot-input {
      flex: 1;
      padding: 12px 16px;
      border: 1.5px solid #d1d8dd;
      border-radius: 12px;
      background: #ffffff;
      color: #2c3e50;
      font-size: 14px;
      transition: border-color 0.2s;
    }
    html[data-theme='dark'] #chatbot-input {
      background: #2a2a2a;
      color: #e8e8e8;
      border: 1.5px solid #3a3a3a;
    }
    #chatbot-input:focus {
      outline: none;
      border-color: #4a90e2;
    }
    #chatbot-send {
      padding: 12px 22px;
      background: linear-gradient(135deg, #4a90e2 0%, #5c6bc0 100%);
      color: white;
      border: none;
      border-radius: 12px;
      cursor: pointer;
      font-weight: 500;
      font-size: 14px;
      transition: all 0.2s;
      box-shadow: 0 2px 6px rgba(74, 144, 226, 0.25);
    }
    #chatbot-send:hover {
      transform: translateY(-1px);
      box-shadow: 0 3px 10px rgba(74, 144, 226, 0.35);
    }
    #chatbot-send:disabled {
      background: #b8bcc4;
      cursor: not-allowed;
      transform: none;
      box-shadow: none;
    }
  `;

  const styleSheet = document.createElement('style');
  styleSheet.textContent = styles;
  document.head.appendChild(styleSheet);

  const chatbotHTML = `
    <button id="chatbot-button">💬</button>
    <div id="chatbot-panel">
      <div id="chatbot-header">
        <span>Ask about the book</span>
        <button id="chatbot-close">✕</button>
      </div>
      <div id="chatbot-messages"></div>
      <div id="chatbot-input-container">
        <input type="text" id="chatbot-input" placeholder="Ask a question..." />
        <button id="chatbot-send">Send</button>
      </div>
    </div>
  `;

  document.body.insertAdjacentHTML('beforeend', chatbotHTML);

  const button = document.getElementById('chatbot-button');
  const panel = document.getElementById('chatbot-panel');
  const closeBtn = document.getElementById('chatbot-close');
  const input = document.getElementById('chatbot-input');
  const sendBtn = document.getElementById('chatbot-send');
  const messagesContainer = document.getElementById('chatbot-messages');

  button.addEventListener('click', () => {
    panel.classList.toggle('open');
    if (panel.classList.contains('open')) {
      input.focus();
    }
  });

  closeBtn.addEventListener('click', () => {
    panel.classList.remove('open');
  });

  async function sendMessage() {
    const question = input.value.trim();
    if (!question) return;

    addMessage(question, 'user');
    input.value = '';
    sendBtn.disabled = true;

    const loadingMsg = addMessage('Thinking...', 'loading');

    try {
      const selectedText = window.getSelection().toString().trim();

      const response = await fetch(API_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          user_question: question,
          selected_text: selectedText || null
        })
      });

      if (!response.ok) {
        throw new Error(`Server error: ${response.status}`);
      }

      const data = await response.json();
      loadingMsg.remove();

      const answer = data.answer;
      const sources = data.sources && data.sources.length > 0
        ? `<div class="chat-sources">Sources: ${data.sources.map(s => s.source).join(', ')}</div>`
        : '';

      addMessageHTML(answer + sources, 'bot');

    } catch (error) {
      loadingMsg.remove();
      addMessage(`I'm having trouble connecting to the server. Please try again in a moment.`, 'bot');
    }

    sendBtn.disabled = false;
  }

  function addMessage(text, type) {
    const msg = document.createElement('div');
    msg.className = `chat-message ${type}`;
    msg.textContent = text;
    messagesContainer.appendChild(msg);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
    return msg;
  }

  function addMessageHTML(html, type) {
    const msg = document.createElement('div');
    msg.className = `chat-message ${type}`;
    msg.innerHTML = html;
    messagesContainer.appendChild(msg);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
    return msg;
  }

  sendBtn.addEventListener('click', sendMessage);
  input.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') sendMessage();
  });
})();
