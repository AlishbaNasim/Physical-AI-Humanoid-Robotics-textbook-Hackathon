module.exports = function plugin(context, options) {
  return {
    name: 'chatbot-inject-plugin',

    getClientModules() {
      return [require.resolve('./ChatbotInjector.jsx')];
    },
  };
};