# Tasks: RAG Agent with OpenAI Agents SDK

**Feature**: RAG Agent Construction with OpenAI Agents SDK
**Branch**: `009-rag-agent-construction`
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)
**Created**: 2025-12-16

## Summary

This document outlines the implementation tasks for the RAG (Retrieval-Augmented Generation) agent using the OpenAI Agents SDK. The agent will connect to the existing Qdrant Cloud collection containing embedded book content, accept user queries, retrieve relevant content chunks using vector similarity search, and generate contextual answers using the Gemini 1.5 Flash LLM. The agent will be exposed via a FastAPI endpoint and utilize the context 7 MCP architecture for agent runtime.

## Dependencies

- All user stories depend on successful completion of Phase 1 (Setup) and Phase 2 (Foundational)
- User Story 2 (follow-up questions) depends on User Story 1 (basic querying)
- User Story 3 (citations) depends on User Story 1 (basic retrieval)

## Implementation Strategy

The implementation will follow an MVP-first approach, with User Story 1 (P1) forming the core functionality. Each user story will be developed as an independent, testable increment. The agent will be built with comprehensive logging and metrics collection to enable detailed analysis of retrieval performance and answer quality.

## Parallel Execution Examples

- T002-T004 [P] can be executed in parallel during setup phase
- T007-T009 [P] can be executed in parallel during foundational phase
- Within each user story phase, service implementation and validation can run in parallel

---

## Phase 1: Setup

Setup tasks for project initialization and dependency management.

### Independent Test Criteria

- Project structure is created with all required files and directories
- Dependencies are installed and accessible
- Environment variables are properly configured and validated

- [x] T001 Create project structure in backend/src/rag_agent/
- [x] T002 [P] Install and verify openai-agents-sdk dependency in backend/requirements.txt
- [x] T003 [P] Install and verify google-generativeai dependency in backend/requirements.txt
- [x] T004 [P] Install and verify fastapi and uvicorn dependencies in backend/requirements.txt
- [x] T005 Update backend/.env with GEMINI_API_KEY and GEMINI_MODEL_NAME=Gemini 1.5 Flash
- [x] T006 Create backend/main.py FastAPI application entry point

---

## Phase 2: Foundational

Foundational tasks that provide blocking prerequisites for all user stories.

### Independent Test Criteria

- OpenAI Agents SDK is configured with context 7 MCP servers
- Google Gemini client is initialized and can generate responses
- Qdrant client connection is established and validated
- Agent state management is implemented

- [x] T007 [P] Create agent configuration in backend/src/rag_agent/config.py
- [x] T008 [P] Implement Google Gemini client in backend/src/rag_agent/llm_service.py
- [x] T009 [P] Create agent state models in backend/src/rag_agent/models.py based on data-model.md
- [x] T010 Configure OpenAI Agents SDK with context 7 MCP servers
- [x] T011 Create retrieval tool model based on data-model.md in backend/src/rag_agent/models.py
- [x] T012 Implement environment variable loading and validation
- [x] T013 Create query request/response models based on data-model.md

---

## Phase 3: [US1] Build Core RAG Agent

Implement the core RAG agent functionality that can answer book-related questions using vector retrieval.

### Independent Test Criteria

- Agent receives natural language queries about book content
- Agent calls vector retrieval tool to fetch relevant content chunks
- Agent generates accurate answers based on retrieved content
- Agent returns responses with proper formatting via API

- [x] T014 [P] [US1] Implement agent core logic in backend/src/rag_agent/agent.py
- [x] T015 [P] [US1] Create retrieval tool integration in backend/src/rag_agent/retrieval_tool.py
- [x] T016 [US1] Configure Gemini 1.5 Flash as the agent's LLM
- [x] T017 [US1] Implement tool calling mechanism for vector retrieval
- [x] T018 [US1] Add conversation context management to agent
- [x] T019 [US1] Implement basic query validation and error handling
- [x] T020 [US1] Create basic validation test for agent responses
- [x] T021 [US1] Implement manual validation helper for response quality assessment

---

## Phase 4: [US2] Integrate Retrieval Pipeline

Connect the agent to the existing Qdrant retrieval pipeline for accessing book content.

### Independent Test Criteria

- Agent successfully connects to Qdrant Cloud collection
- Vector retrieval tool returns relevant content chunks with preserved metadata
- Retrieved content is properly formatted for LLM consumption
- Source attribution is maintained throughout the process

- [x] T022 [P] [US2] Enhance retrieval tool to connect to existing Qdrant service
- [x] T023 [P] [US2] Implement content chunk formatting for LLM context
- [x] T024 [US2] Create metadata preservation validation for retrieved content
- [x] T025 [US2] Add retrieval result integration to agent workflow
- [x] T026 [US2] Implement source attribution verification functionality
- [x] T027 [US2] Create retrieval accuracy metrics collection
- [ ] T028 [US2] Add retrieval validation to agent testing
- [ ] T029 [US2] Implement comprehensive retrieval integration test

---

## Phase 5: [US3] Expose Agent via API

Expose the agent functionality via FastAPI endpoints for external consumption.

### Independent Test Criteria

- POST /query endpoint accepts agent requests and returns structured responses
- GET /health endpoint provides agent status and metrics
- API responses match the contract specifications
- Error handling and validation work as specified

- [x] T030 [P] [US3] Create agent API service in backend/src/rag_agent/api_service.py
- [x] T031 [P] [US3] Implement POST /query endpoint per contracts/agent-api.yaml
- [x] T032 [US3] Implement GET /health endpoint per contracts/agent-api.yaml
- [x] T033 [US3] Add request/response validation based on contract specifications
- [x] T034 [US3] Implement API error handling and response formatting
- [ ] T035 [US3] Create API integration tests
- [ ] T036 [US3] Add rate limiting and concurrency handling to API
- [ ] T037 [US3] Implement API logging and monitoring

---

## Phase 6: Polish & Cross-Cutting Concerns

Final implementation tasks including comprehensive testing, documentation, and validation.

### Independent Test Criteria

- Agent successfully answers 90% of book-related questions with accurate responses
- Query response time remains under 5 seconds for 95% of requests
- Agent correctly cites source material for 95% of claims in responses
- System handles 100 concurrent requests during stress testing

- [x] T038 [P] Create comprehensive agent metrics collection (AgentMetrics model)
- [x] T039 [P] Implement AgentResult model for tracking agent interactions
- [x] T040 Add comprehensive logging for agent interactions and metrics
- [x] T041 Create 100-query stress test as per success criteria
- [x] T042 Implement performance monitoring and response time tracking
- [x] T043 Add agent result reporting and summary functionality
- [x] T044 Create final agent validation report generation
- [x] T045 Update quickstart.md with new agent instructions
- [x] T046 Run full validation pipeline and verify all success criteria are met
- [x] T047 Update requirements.txt with any additional dependencies
- [x] T048 Create agent implementation summary documentation