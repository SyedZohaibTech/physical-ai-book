<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0 (MAJOR: Complete project shift from landing page to RAG chatbot)
- Modified principles: All principles updated for RAG chatbot project
- Added sections: RAG Architecture, Text Selection Feature, Performance Requirements
- Removed sections: Landing page specific principles
- Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: N/A (no command templates found)
- Follow-up TODOs: None
-->
# Interactive RAG Chatbot for Physical AI Textbook Constitution

## Core Principles

### I. RAG Architecture-First
The chatbot must implement Retrieval-Augmented Generation (RAG) architecture, retrieving relevant content from vector database before generating answers with LLM. This ensures all responses are grounded in the textbook content rather than hallucinated.

### II. Dual Mode Functionality
The system must support two distinct interaction modes: General Q&A (searching entire book) and Text Selection Q&A (answering specific questions about selected passages). Both modes must provide accurate, contextually relevant responses.

### III. Tech Stack Consistency
Backend must use FastAPI with Python, vector database must be Qdrant Cloud, SQL database must be Neon Serverless Postgres, embeddings must use OpenAI text-embedding-3-small, and LLM responses must use OpenAI GPT-3.5-turbo via ChatKit SDK. Frontend must be React component integrated with Docusaurus.

### IV. Embedded Experience
The chatbot interface must be seamlessly embedded in the Docusaurus site as a floating button in the bottom-right corner that expands to a chat window. This ensures consistent user experience across all textbook pages without disrupting the reading flow.

### V. Production-Ready Quality
All components must include comprehensive error handling, loading states, API rate limiting, and proper logging. The system must gracefully handle failures and provide meaningful feedback to users when issues occur.

### VI. Performance Optimization
Response times must meet strict requirements: vector search under 500ms and LLM response under 3 seconds, resulting in total response time under 5 seconds. All components must be optimized for these performance targets.

## Additional Requirements

### Technical Requirements
- Backend: FastAPI (Python) with async support for handling concurrent requests
- Vector DB: Qdrant Cloud free tier for document embeddings and retrieval
- SQL DB: Neon Serverless Postgres for session management and user data
- Embeddings: OpenAI text-embedding-3-small for creating document vectors
- LLM: OpenAI GPT-3.5-turbo via ChatKit SDK for answer generation
- Frontend: React component integrated with Docusaurus documentation site
- Integration: Floating chat button that appears on all textbook pages
- Security: API rate limiting and authentication where appropriate

### UI/UX Requirements
- Clean, modern chat interface that matches textbook's visual design
- Smooth animations for expanding/collapsing chat window
- Clear visual indicators for loading states
- Intuitive text selection interaction flow
- Accessible design compliant with WCAG 2.1 AA standards
- Mobile-responsive chat interface

### Content Requirements
- Accurate retrieval of relevant textbook content based on user queries
- Proper attribution of information sources to specific textbook sections
- Ability to handle complex questions requiring multiple content pieces
- Support for follow-up questions within conversation context

## Development Workflow

### Quality Gates
- All components must pass performance benchmarks (response times under 5s)
- Accuracy verification through content matching tests
- Cross-browser compatibility testing (Chrome, Firefox, Safari, Edge)
- Mobile responsiveness verification
- Accessibility compliance checks (WCAG 2.1 AA standards)

### Review Process
- Architecture review focusing on RAG implementation before development
- Performance audit to ensure response time requirements
- Content accuracy verification comparing responses to textbook
- User experience review for both chat modes
- Security review for API endpoints and rate limiting

### Deployment Policy
- Vector database must be populated with complete textbook content
- Performance metrics must meet established requirements
- All chat functionality must be fully tested
- Error handling verified under various failure conditions

## Governance

This constitution supersedes all other practices for the Interactive RAG Chatbot for Physical AI Textbook project. All PRs and reviews must verify compliance with these principles. Amendments require documentation of the change, approval from project stakeholders, and a migration plan if needed.

**Version**: 2.0.0 | **Ratified**: 2025-12-24 | **Last Amended**: 2025-12-24