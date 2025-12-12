---
name: "Update Project Constitution"
description: "Update project constitution for Physical AI & Humanoid Robotics"
tools: []
---

## User Input

```
Project Title

Physical AI & Humanoid Robotics — Spec-Driven Book + Integrated RAG Chatbot + Personalization System

Core Principles

Accuracy through primary source verification
All robotics, AI, ML, control systems, and architecture claims must be verified using primary sources and official documentation.

Clarity for an academic audience
Content must be understandable to readers with a computer science background, written at a Flesch-Kincaid Grade 10–12 level.

Reproducibility
All examples, RAG pipelines, API calls, diagrams, and implementations must be reproducible and correctly reflect working code.

Rigor (peer-reviewed sources preferred)
At least 50% of references must be peer-reviewed robotics/AI/HCI research papers or textbooks.

Key Standards

Traceability
All factual claims must be directly traceable to cited sources.

Citation Format
APA style for all academic, technical, and documentation citations.

Source Requirements

Minimum 15 credible sources

At least 50% peer-reviewed

Plagiarism Policy
0% tolerance for plagiarism of any form.

Writing Quality
Clear, structured, academic writing.

Book Requirements (AI/Spec-Driven Creation)

Write a complete technical book using Docusaurus.

Use Spec-Kit Plus and Claude Code for structured, spec-driven writing.

Deploy the final book to GitHub Pages.

Word Count: 5,000–7,000 words.

Produce a PDF with embedded APA citations.

Include high-quality diagrams, reproducible code blocks, and clear technical explanations.

RAG Chatbot Requirements (Integrated Into the Book)

A working RAG chatbot must be embedded into the published Docusaurus book.

Required Technologies

OpenAI Agents / ChatKit SDK

FastAPI backend

Neon Serverless Postgres (auth, user profile, chat history)

Qdrant Cloud Free Tier (vector embeddings)

Functional Requirements

Answer user questions based only on the book content.

Support selected-text-only answering mode.

Provide source citations or relevant chunks.

Maintain users' chat history in Postgres.

Must avoid hallucinations by grounding every answer in retrieved context.

Bonus Features (Optional – 50 Points Each)
1. Better-Auth Signup & Signin

Implement full authentication using better-auth.

At signup, ask users about:

software background

hardware background

Store responses in Postgres to support future personalization.

2. Personalized Chapter Content

Add a "Personalize Content" button at the start of each chapter.

Logged-in users can toggle content difficulty:

beginner level

intermediate

expert/advanced

Personalization should use user background stored in the database.

3. Urdu Translation Button

Add an "Urdu Translate" button at the start of each chapter.

Use OpenAI Agents to translate chapter content into Urdu.

Code blocks, formulas, and diagrams must remain intact.

Constraints

Use Docusaurus for the book.

Use Spec-Kit Plus for all specifications and project structure.

Use FastAPI for backend, Qdrant for embeddings, and Neon Postgres for user/auth data.

Use TypeScript + React for UI components.

All diagrams must accurately reflect the implemented pipeline.
```

## Response Summary

Updated the project constitution at `.specify/memory/constitution.md` with all the required principles and standards for the Physical AI & Humanoid Robotics book project. The new constitution includes:

- Core principles: Accuracy through primary source verification, Clarity for academic audience, Reproducibility, Rigor with peer-reviewed sources, Traceability, and APA citation format
- Key Standards: Source requirements, plagiarism policy, and writing quality standards
- Development Workflow: Technology stack requirements and structural guidelines
- Governance: Compliance and review expectations

The constitution now properly reflects all project requirements including focus on primary source verification, academic writing standards, reproducibility, and the technology stack for the Docusaurus book, RAG chatbot, and personalization features.

## Outcome

The project constitution has been successfully created with version 1.0.0, establishing foundational principles and standards for the Physical AI & Humanoid Robotics book project, including the book creation, RAG chatbot implementation, and personalization system.

## Evaluation

The constitution update successfully incorporates all the specified requirements. All placeholder tokens were replaced with concrete values derived from the user input. The file was validated to ensure proper format and no remaining unexplained bracket tokens. Dependent templates were checked for consistency and found to be compatible with the new constitution.