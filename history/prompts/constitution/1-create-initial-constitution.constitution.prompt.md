---
id: 1
title: Create Initial Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, spec-first, ai-book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - null
---

## Prompt

```text
/sp.constitution
Project: AI/Spec-Driven Book with Embedded RAG Chatbot
Core principles:
* Spec-first workflow using Spec-Kit Plus
* Technical accuracy from official sources
* Clear, developer-focused writing
* Reproducible setup and deployment
Key standards:
* Book written with Docusaurus and deployed on GitHub Pages
* RAG chatbot grounded only in book content or user-selected text
* Stack: OpenAI Agents/EhatKit, FastAPI, Neon Postgres, Qdrant Cloud
* Runnable, well-documented code
Constraints:
-GitHub-based source control
* No hallucinated responses
* End-to-end reproducibility                                                                                                                                                                                                                                               Success criteria:
* Live book on GitHub Pages
* Fully functional embedded RAG chatbot
* All specs implemented via Spec-Kit Plus
```

## Response snapshot

Created initial constitution for the AI/Spec-Driven Book with Embedded RAG Chatbot project, incorporating the specified core principles and standards. The constitution includes six core principles:

1. Spec-First Workflow (NON-NEGOTIABLE)
2. Technical Accuracy and Official Sources
3. Developer-Focused Writing
4. Reproducible Setup and Deployment
5. Grounded RAG Responses
6. End-to-End Testability

Updated template files to align with the new constitution:
- Modified plan-template.md to include constitution check gates
- Enhanced spec-template.md with technical accuracy verification and reproducibility requirements sections
- Updated tasks-template.md to include constitution alignment and specific tasks for technical accuracy and reproducible setup

## Outcome

- ✅ Impact: Created foundational project constitution and aligned templates
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md and 3 template files
- 🔁 Next prompts: Continue with spec creation for features
- 🧠 Reflection: Constitution provides clear governance framework for project decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A