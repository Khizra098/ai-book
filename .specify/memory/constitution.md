<!--
Sync Impact Report:
- Version change: 0.1.0 → 1.0.0
- Modified principles: All placeholders replaced with project-specific content
- Added sections: Core principles aligned with AI/Spec-Driven Book project
- Removed sections: Template comments
- Templates requiring updates: N/A (initial constitution)
- Follow-up TODOs: None
-->
# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Workflow (NON-NEGOTIABLE)
All development begins with comprehensive specifications using Spec-Kit Plus; Every feature, requirement, and acceptance criterion must be documented in spec files before implementation; Changes to functionality require spec updates first, then implementation follows.

### II. Technical Accuracy and Official Sources
All content and code examples must be verified against official documentation and authoritative sources; No hallucinated information or unverified claims allowed; Citations and references required for all technical assertions.

### III. Developer-Focused Writing
Documentation must be clear, actionable, and immediately applicable to real-world scenarios; Complex concepts broken down into digestible, step-by-step instructions; Code examples must be runnable and well-documented.

### IV. Reproducible Setup and Deployment
Every setup, installation, and deployment process must be fully reproducible from scratch; Detailed prerequisites, environment setup, and deployment guides required; Infrastructure as code wherever possible.

### V. Grounded RAG Responses
The embedded chatbot must only respond based on book content or user-selected text; No hallucinated responses or fabricated information; Strict grounding in provided context documents.

### VI. End-to-End Testability
All features must include comprehensive testing coverage; Integration tests required for the full book-RAG-chatbot pipeline; Automated testing for deployment and functionality.

## Technology Stack and Standards
The project follows a modern, cloud-native technology stack with specific component choices to ensure consistency and maintainability.

### Stack Requirements:
- Book platform: Docusaurus for documentation and static site generation
- Deployment: GitHub Pages for hosting and distribution
- RAG backend: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud
- Code quality: Well-documented, commented, and tested implementations
- Source control: GitHub-based with proper branching and PR workflows

### Architecture Standards:
- API design: RESTful interfaces with proper error handling and documentation
- Database schema: Proper normalization, indexing, and migration strategies
- Search functionality: Full-text search capabilities with proper relevance ranking
- Security: Authentication, authorization, and data protection mechanisms
- Performance: Optimized queries, caching strategies, and resource management

## Development Workflow and Quality Assurance
The team follows a disciplined development process that ensures high-quality deliverables and consistent progress.

### Workflow Requirements:
- Specification-first: All features planned and documented in specs/ directory before implementation
- Branch strategy: Feature branches from main, pull requests with code review
- Testing: Unit, integration, and end-to-end tests for all functionality
- Documentation: Inline code comments, API documentation, and user guides
- Code reviews: Mandatory peer review for all pull requests
- Continuous integration: Automated builds, tests, and deployment validation

### Quality Gates:
- All tests must pass before merging
- Code coverage thresholds maintained
- Security scanning passes
- Performance benchmarks met
- Documentation completeness verified

## Governance
This constitution serves as the authoritative guide for all project decisions and practices. All team members must comply with these principles, and any deviations require formal amendment procedures.

### Amendment Process:
- Proposed changes must be documented with rationale
- Team consensus required for approval
- Impact assessment on existing artifacts required
- Version updates must follow semantic versioning

### Compliance:
- All pull requests must demonstrate constitution compliance
- Regular audits ensure ongoing adherence
- Training provided for new team members

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
