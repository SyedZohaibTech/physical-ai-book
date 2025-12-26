# Quickstart Guide: Interactive RAG Chatbot for Physical AI Textbook

## Prerequisites
- Python 3.11+
- Node.js 16+ (for Docusaurus)
- Access to OpenAI API
- Qdrant Cloud account
- Neon Postgres account

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cp .env.example .env
# Edit .env with your API keys and connection strings
```

### 3. Environment Variables
Create a `.env` file with the following variables:
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
```

### 4. Initialize Vector Store
```bash
# Run the embedding script to populate Qdrant with textbook content
python embed_book.py
```

### 5. Start Backend Server
```bash
# From the backend directory
uvicorn main:app --reload
```

### 6. Frontend Setup
```bash
# In a new terminal, navigate to website directory
cd website

# Install dependencies
npm install

# Start the Docusaurus development server
npm start
```

## Usage

### General Q&A
1. Visit the textbook website
2. Click the floating chat button in the bottom-right corner
3. Type your question about the textbook content
4. Receive an AI-generated response with sources cited

### Text Selection Q&A
1. Select text in the textbook
2. Click the "Ask about selection" button that appears
3. Ask a specific question about the selected text
4. Receive a response focused on the selected content

## API Endpoints

### Chat Endpoint
`POST /api/chat`
- Send user messages and receive AI responses
- Supports conversation history with conversation_id

### Health Check
`GET /api/health`
- Check the health status of services
- Returns status of Qdrant and Postgres connections

## Troubleshooting

### Common Issues
- Ensure all environment variables are set correctly
- Verify Qdrant and Postgres connections
- Check that the embedding script has run successfully
- Confirm OpenAI API key has sufficient quota