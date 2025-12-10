module.exports = function () {
  return {
    name: 'chatbot-inject',
    injectHtmlTags() {
      return {
        headTags: [],
        postBodyTags: [
          {
            tagName: 'script',
            innerHTML: `
              console.log("Chatbot Loaded");
            `,
          },
        ],
      };
    },
  };
};
