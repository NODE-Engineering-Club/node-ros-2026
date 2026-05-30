// Build configuration for the Foxglove extension.
//
// The `foxglove-extension` CLI (from create-foxglove-extension) drives the build
// with webpack under the hood. This file lets us customize that webpack config —
// it is the extension equivalent of a hand-written `webpack.config.js`.
//
// The exported `webpack` hook receives the CLI's default webpack config and must
// return a (possibly modified) config. We add a rule so that CSS imports (e.g.
// Leaflet's stylesheet) are processed by css-loader and injected at runtime by
// style-loader, instead of falling through to ts-loader.

import type { Configuration } from "webpack";

export default {
  webpack: (config: Configuration): Configuration => {
    config.module ??= {};
    config.module.rules ??= [];
    config.module.rules.push({
      test: /\.css$/,
      use: ["style-loader", "css-loader"],
    });

    return config;
  },
};
