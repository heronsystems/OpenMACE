var path = require("path");

var config = {
  /*
   * app.ts represents the entry point to your web application. Webpack will
   * recursively go through every "require" statement in app.ts and
   * efficiently build out the application's dependency tree.
   */
  entry: ["./app/src/app.tsx"],

  /*
   * The combination of path and filename tells Webpack what name to give to
   * the final bundled JavaScript file and where to store this file.
   */
  output: {
      filename: 'bundle.js',
      path: __dirname + '/app/build',
      publicPath: `http://localhost:${process.env.PORT}/app/build`
    },

  devServer:{
        contentBase: 'html',
        port: process.env.PORT
  },

  /*
   * resolve lets Webpack now in advance what file extensions you plan on
   * "require"ing into the web application, and allows you to drop them
   * in your code.
   */
  resolve: {
    extensions: [".ts", ".tsx", ".js"]
  },

  performance: { hints: false },
  node: {
    fs: "empty"
  },
  module: {
    /*
     * Each loader needs an associated Regex test that goes through each
     * of the files you've included (or in this case, all files but the
     * ones in the excluded directories) and finds all files that pass
     * the test. Then it will apply the loader to that file. I haven't
     * installed ts-loader yet, but will do that shortly.
     */
    loaders: [
      {
        test: /\.tsx?$/,
        loader: "babel-loader!ts-loader",
        exclude: /node_modules/
      },
      {
        test: /\.js$/,
        loader: "babel-loader",
        exclude: /node_modules/,
        include: path.join(__dirname, 'src')
      },
      {
        test: /\.css$/,
        loader: "style-loader!css-loader"
      },
      {
        test: /\.(png|woff|woff2|eot|ttf|svg)$/, loader: 'url-loader?limit=100000'
      }
    ]
  }
};
// require('electron-reload')(__dirname);
module.exports = config;