// @ts-check

/**
 * @param {import('@docusaurus/types').DocusaurusConfig} context
 * @param {Object} opts
 */
module.exports = function ragChatbotPlugin(context, opts = {}) {
  return {
    name: 'rag-chatbot-plugin',

    getClientModules() {
      return [require.resolve('./rag-chatbot-injector')];
    },

    injectHtmlTags() {
      return {
        // Add the script to load the chatbot component
        postBodyTags: [
          `<div id="rag-chatbot-container" style="position: fixed; bottom: 20px; right: 20px; z-index: 1000;"></div>`,
        ],
      };
    },
  };
};