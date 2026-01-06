# Data Model: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade
**Created**: 2025-12-17
**Status**: Complete

## Theme Configuration

### ThemeConfig
- **id**: string (primary key)
- **name**: string (e.g., "robotics-dark", "robotics-light")
- **colors**: object
  - primary: string (hex/rgb)
  - secondary: string (hex/rgb)
  - accentColors: array[string] (4 module-specific colors)
  - background: string
  - text: string
  - textSecondary: string
- **typography**: object
  - fontFamily: string (e.g., "Inter")
  - codeFontFamily: string (e.g., "JetBrains Mono")
  - fontSize: object (base sizes)
- **animation**: object
  - enabled: boolean
  - heroAnimation: string (animation type)
  - transitionSpeed: string

## User Progress Tracking

### ProgressData
- **id**: string (userId or session-based)
- **modules**: object
  - moduleId: object
    - sections: object
      - sectionId: object
        - completed: boolean
        - startedAt: date
        - completedAt: date
        - timeSpent: number (seconds)
- **lastAccessed**: date
- **preferences**: object
  - theme: string
  - accentColor: string
  - reducedMotion: boolean

## Content Enhancement Metadata

### ContentMetadata
- **pageId**: string (URL-based)
- **moduleId**: string
- **estimatedReadingTime**: number (minutes)
- **difficultyLevel**: enum ("beginner", "intermediate", "advanced")
- **contentFeatures**: object
  - hasCodeBlocks: boolean
  - hasImages: boolean
  - hasVideos: boolean
  - hasExpandableSections: boolean
- **createdAt**: date
- **updatedAt**: date

## Navigation Structure

### NavigationItem
- **id**: string
- **title**: string
- **url**: string
- **moduleId**: string
- **parentId**: string (nullable)
- **order**: number
- **children**: array[NavigationItem]
- **progress**: object
  - completed: boolean
  - totalSections: number
  - completedSections: number

## Search Enhancement

### SearchIndex
- **id**: string
- **title**: string
- **contentPreview**: string
- **url**: string
- **moduleId**: string
- **tags**: array[string]
- **lastIndexed**: date

## UI Component States

### ComponentState
- **componentId**: string (e.g., "code-block-123", "expandable-section-456")
- **type**: enum ("codeBlock", "expandableSection", "imageZoom", "videoEmbed")
- **state**: object
  - expanded: boolean (for expandable sections)
  - copied: boolean (for code blocks)
  - zoomLevel: number (for image zoom)
- **userId**: string (nullable, for persistent states)
- **sessionId**: string
- **lastUpdated**: date

## Animation Preferences

### AnimationSettings
- **id**: string
- **userId**: string (nullable, for logged in users)
- **sessionId**: string
- **reducedMotion**: boolean
- **animationSpeed**: enum ("slow", "normal", "fast")
- **enabledAnimations**: object
  - heroAnimations: boolean
  - navigationTransitions: boolean
  - contentAnimations: boolean
- **updatedAt**: date

## Homepage Configuration

### HomepageConfig
- **id**: string
- **background**: object
  - color: string (e.g., "#0a3d2e" for dark green)
  - image: string (path to humanoid robot image)
  - imageAlt: string (alt text for accessibility)
- **title**: string ("Physical AI & Humanoid Robotics")
- **ctaButton**: object
  - text: string ("Start Reading Book")
  - color: string (e.g., green)
  - link: string (URL for the button)
  - style: object (CSS classes or styling properties)
- **layout**: object
  - order: number (position of elements)
  - responsiveness: object (breakpoints and mobile behavior)
- **createdAt**: date
- **updatedAt**: date

## Module Configuration

### ModuleConfig
- **id**: string
- **name**: string
- **displayName**: string
- **accentColor**: string (hex value)
- **order**: number
- **sections**: array[object]
  - id: string
  - title: string
  - url: string
  - difficulty: enum ("beginner", "intermediate", "advanced")
  - estimatedTime: number (minutes)
- **createdAt**: date
- **updatedAt**: date