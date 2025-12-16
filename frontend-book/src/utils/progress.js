// Client-side progress tracking using localStorage
class ProgressTracker {
  constructor() {
    this.storageKey = 'docusaurus-progress';
  }

  getProgress() {
    try {
      const data = localStorage.getItem(this.storageKey);
      return data ? JSON.parse(data) : this.getDefaultProgress();
    } catch (error) {
      console.error('Error reading progress data:', error);
      return this.getDefaultProgress();
    }
  }

  updateProgress(moduleId, sectionId, completed) {
    const progress = this.getProgress();
    if (!progress.modules[moduleId]) {
      progress.modules[moduleId] = { sections: {} };
    }

    const now = new Date().toISOString();
    progress.modules[moduleId].sections[sectionId] = {
      completed,
      startedAt: now,
      completedAt: completed ? now : null,
      timeSpent: 0
    };

    try {
      localStorage.setItem(this.storageKey, JSON.stringify(progress));
      return true;
    } catch (error) {
      console.error('Error saving progress data:', error);
      return false;
    }
  }

  // Enhanced method for section completion tracking
  trackSectionCompletion(moduleId, sectionId) {
    const progress = this.getProgress();
    if (!progress.modules[moduleId]) {
      progress.modules[moduleId] = { sections: {} };
    }

    const section = progress.modules[moduleId].sections[sectionId];
    if (!section || !section.completed) {
      const now = new Date().toISOString();
      progress.modules[moduleId].sections[sectionId] = {
        completed: true,
        startedAt: section?.startedAt || now,
        completedAt: now,
        timeSpent: section?.timeSpent || 0
      };

      try {
        localStorage.setItem(this.storageKey, JSON.stringify(progress));
        return true;
      } catch (error) {
        console.error('Error saving progress data:', error);
        return false;
      }
    }
    return true;
  }

  // Method to calculate progress percentage
  calculateModuleProgress(moduleId) {
    const progress = this.getProgress();
    const moduleProgress = progress.modules[moduleId];

    if (!moduleProgress || !moduleProgress.sections) {
      return 0;
    }

    const sections = Object.values(moduleProgress.sections);
    const totalSections = sections.length;
    const completedSections = sections.filter(section => section.completed).length;

    return totalSections > 0 ? Math.round((completedSections / totalSections) * 100) : 0;
  }

  // Method to get progress indicators for navigation items
  getSectionProgress(moduleId, sectionId) {
    const progress = this.getProgress();
    const section = progress.modules[moduleId]?.sections[sectionId];
    return section || { completed: false, startedAt: null, completedAt: null, timeSpent: 0 };
  }

  getDefaultProgress() {
    return {
      modules: {},
      lastAccessed: new Date().toISOString(),
      preferences: {
        theme: 'dark',
        accentColor: 'module1'
      }
    };
  }
}

export default new ProgressTracker();