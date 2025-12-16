// Reading time estimation utility
// Based on average reading speed of 200 words per minute

const calculateReadingTime = (text) => {
  if (!text || typeof text !== 'string') {
    return 0;
  }

  // Remove HTML tags and extra whitespace
  const cleanText = text.replace(/<[^>]*>/g, ' ').replace(/\s+/g, ' ').trim();

  // Count words (approximately)
  const wordCount = cleanText.split(/\s+/).filter(word => word.length > 0).length;

  // Calculate reading time in minutes (rounded up)
  const minutes = Math.ceil(wordCount / 200);

  return Math.max(1, minutes); // Minimum of 1 minute
};

// Export the function
export default calculateReadingTime;

// If used as a Node.js module for server-side calculation
if (typeof module !== 'undefined' && module.exports) {
  module.exports = calculateReadingTime;
}