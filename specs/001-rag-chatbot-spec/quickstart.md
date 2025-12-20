# Quickstart: RAG Chatbot

This guide provides a brief overview of how to get the RAG chatbot project up and running locally.

## Prerequisites

- Python 3.11
- Node.js and npm
- Access to OpenAI, Qdrant, and Neon API keys

## Backend Setup

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Create and activate a virtual environment**:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

3.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

4.  **Set up environment variables**:
    -   Copy the `.env.example` file to `.env`.
    -   Fill in your API keys and database URL.

5.  **Run the development server**:
    ```bash
    uvicorn main:app --reload
    ```
    The backend will be available at `http://127.0.0.1:8000`.

## Frontend Setup

The frontend is part of the existing Docusaurus website.

1.  **Navigate to the website directory**:
    ```bash
    cd website
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```

3.  **Run the development server**:
    ```bash
    npm start
    ```
    The website will be available at `http://localhost:3000`.

## Content Indexing

To populate the Qdrant vector database with the book's content, run the indexing script.

1.  **Navigate to the project root**.
2.  **Ensure your backend virtual environment is activated**.
3.  **Run the script**:
    ```bash
    python scripts/index_book_content.py
    ```
