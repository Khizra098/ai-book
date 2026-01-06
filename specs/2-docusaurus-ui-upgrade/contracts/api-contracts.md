# API Contracts: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade
**Created**: 2025-12-17
**Status**: Complete

## Theme API

### Switch Theme
- **Endpoint**: `POST /api/theme/switch`
- **Purpose**: Switch between light/dark mode and update theme preferences
- **Authentication**: None (client-side storage)
- **Request Body**:
```json
{
  "theme": {
    "type": "dark|light",
    "accentColor": "module1|module2|module3|module4",
    "reducedMotion": "boolean"
  }
}
```
- **Response**:
```json
{
  "success": "boolean",
  "appliedTheme": {
    "type": "string",
    "accentColor": "string",
    "reducedMotion": "boolean"
  },
  "timestamp": "date"
}
```
- **Error Responses**:
  - 400: Invalid theme configuration
  - 500: Theme application failed

### Get Current Theme
- **Endpoint**: `GET /api/theme/current`
- **Purpose**: Retrieve current theme configuration
- **Authentication**: None
- **Response**:
```json
{
  "theme": {
    "type": "dark|light",
    "accentColor": "string",
    "reducedMotion": "boolean"
  },
  "timestamp": "date"
}
```

## Progress Tracking API

### Get User Progress
- **Endpoint**: `GET /api/progress/user`
- **Purpose**: Retrieve user progress data across all modules
- **Authentication**: None (uses client-side storage)
- **Response**:
```json
{
  "progress": {
    "modules": {
      "moduleId": {
        "sections": {
          "sectionId": {
            "completed": "boolean",
            "startedAt": "date",
            "completedAt": "date",
            "timeSpent": "number"
          }
        }
      }
    },
    "lastAccessed": "date",
    "preferences": {
      "theme": "string",
      "accentColor": "string",
      "reducedMotion": "boolean"
    }
  }
}
```

### Update Section Progress
- **Endpoint**: `POST /api/progress/update`
- **Purpose**: Update progress for a specific section
- **Authentication**: None
- **Request Body**:
```json
{
  "moduleId": "string",
  "sectionId": "string",
  "completed": "boolean",
  "timeSpent": "number"
}
```
- **Response**:
```json
{
  "success": "boolean",
  "updatedProgress": {
    "moduleId": "string",
    "sectionId": "string",
    "completed": "boolean",
    "updatedAt": "date"
  }
}
```
- **Error Responses**:
  - 400: Invalid module or section ID
  - 500: Progress update failed

### Reset Progress
- **Endpoint**: `POST /api/progress/reset`
- **Purpose**: Reset progress for a specific module or all modules
- **Request Body**:
```json
{
  "moduleId": "string|null"  // null to reset all
}
```
- **Response**:
```json
{
  "success": "boolean",
  "resetModule": "string|null",
  "timestamp": "date"
}
```

## Search Enhancement API

### Search Content
- **Endpoint**: `GET /api/search`
- **Purpose**: Search content across all modules with enhanced functionality
- **Query Parameters**:
  - q: search query string
  - moduleId: optional filter by module
  - limit: optional result limit (default 10)
- **Response**:
```json
{
  "query": "string",
  "results": [
    {
      "id": "string",
      "title": "string",
      "contentPreview": "string",
      "url": "string",
      "moduleId": "string",
      "relevance": "number",
      "tags": ["string"]
    }
  ],
  "totalResults": "number",
  "searchTime": "number"  // in milliseconds
}
```

### Get Search Suggestions
- **Endpoint**: `GET /api/search/suggestions`
- **Purpose**: Get search term suggestions as user types
- **Query Parameters**:
  - q: partial search query string
- **Response**:
```json
{
  "query": "string",
  "suggestions": ["string"],
  "timestamp": "date"
}
```

## Content Enhancement API

### Get Content Metadata
- **Endpoint**: `GET /api/content/metadata`
- **Purpose**: Retrieve metadata for a specific page including reading time and difficulty
- **Query Parameters**:
  - url: page URL
- **Response**:
```json
{
  "pageId": "string",
  "moduleId": "string",
  "estimatedReadingTime": "number",
  "difficultyLevel": "beginner|intermediate|advanced",
  "contentFeatures": {
    "hasCodeBlocks": "boolean",
    "hasImages": "boolean",
    "hasVideos": "boolean",
    "hasExpandableSections": "boolean"
  },
  "createdAt": "date",
  "updatedAt": "date"
}
```

### Update Component State
- **Endpoint**: `POST /api/component/state`
- **Purpose**: Update state for UI components like expandable sections or code block copy status
- **Request Body**:
```json
{
  "componentId": "string",
  "type": "codeBlock|expandableSection|imageZoom|videoEmbed",
  "state": {
    "expanded": "boolean",
    "copied": "boolean",
    "zoomLevel": "number"
  },
  "pageUrl": "string"
}
```
- **Response**:
```json
{
  "success": "boolean",
  "componentId": "string",
  "updatedState": "object",
  "timestamp": "date"
}
```

## Homepage API

### Get Homepage Configuration
- **Endpoint**: `GET /api/homepage/config`
- **Purpose**: Retrieve homepage configuration including background, images, and call-to-action settings
- **Authentication**: None
- **Response**:
```json
{
  "id": "string",
  "background": {
    "color": "string",
    "image": "string",
    "imageAlt": "string"
  },
  "title": "string",
  "ctaButton": {
    "text": "string",
    "color": "string",
    "link": "string",
    "style": "object"
  },
  "layout": {
    "order": "number",
    "responsiveness": "object"
  },
  "createdAt": "date",
  "updatedAt": "date"
}
```

## Module Configuration API

### Get Module Configurations
- **Endpoint**: `GET /api/modules/config`
- **Purpose**: Retrieve configuration for all modules including accent colors and section structure
- **Response**:
```json
{
  "modules": [
    {
      "id": "string",
      "name": "string",
      "displayName": "string",
      "accentColor": "string",
      "order": "number",
      "sections": [
        {
          "id": "string",
          "title": "string",
          "url": "string",
          "difficulty": "beginner|intermediate|advanced",
          "estimatedTime": "number"
        }
      ],
      "createdAt": "date",
      "updatedAt": "date"
    }
  ]
}
```