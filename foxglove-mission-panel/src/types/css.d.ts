// Ambient declaration so TypeScript accepts side-effect CSS imports
// (e.g. `import "leaflet/dist/leaflet.css"`). The actual bundling is handled by
// the foxglove-extension webpack build (style-loader + css-loader).
declare module "*.css";
