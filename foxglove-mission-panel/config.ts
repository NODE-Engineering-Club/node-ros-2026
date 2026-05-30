// Build configuration for the Foxglove extension.
//
// The `foxglove-extension` CLI (from @foxglove/extension) drives the build with
// webpack under the hood. This file lets us customize that webpack config — it
// is the extension equivalent of a hand-written `webpack.config.js`.
//
// The exported `webpack` hook receives the CLI's default webpack config and must
// return a (possibly modified) config. Right now we pass it through unchanged;
// the commented block below is the extension point for when we add the map
// library and need to bundle its CSS as injectable strings.

import type { Configuration } from "webpack";

export default {
  webpack: (config: Configuration): Configuration => {
    // --- Extension point: bundle third-party CSS as raw strings -------------
    // When we add a map library (e.g. Leaflet), its stylesheet can be imported
    // as text and injected at runtime so the panel works offline without a CDN:
    //
    //   config.module?.rules?.push({
    //     test: /\.css$/i,
    //     type: "asset/source", // import returns the CSS file contents as a string
    //   });
    //
    // We leave it commented until the map library is chosen so the default
    // build behaviour is not altered prematurely.

    return config;
  },
};
