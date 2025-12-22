// API endpoint for homepage configuration
// This would typically be implemented as a server-side API route
// For Docusaurus, we can create a client-side service that returns the configuration

const getHomepageConfig = () => {
  return {
    id: "main-homepage",
    background: {
      color: "#0a3d2e",
      image: "/img/humanoid-robot.png",
      imageAlt: "Humanoid Robot representing Physical AI & Robotics"
    },
    title: "Physical AI & Humanoid Robotics",
    ctaButton: {
      text: "Start Reading Book",
      color: "#08f45fff",
      link: "/docs/intro",
      style: {
        fontSize: "1.2rem",
        padding: "1rem 2rem",
        backgroundColor: "#4ade80",
        borderColor: "#4ade80"
      }
    },
    layout: {
      order: 1,
      responsiveness: {
        mobileBreakpoint: "996px",
        tabletBreakpoint: "768px"
      }
    },
    createdAt: "2025-12-18T00:00:00.000Z",
    updatedAt: "2025-12-18T00:00:00.000Z"
  };
};

export default getHomepageConfig;