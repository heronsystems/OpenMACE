module.exports = {
    transform: {
        "^.+\\.js$": "<rootDir>/node_modules/babel-jest",
        ".(ts|tsx)": "ts-jest",
        "\\.(jpg|jpeg|png|gif|eot|otf|webp|svg|ttf|woff|woff2|mp4|webm|wav|mp3|m4a|aac|oga)$":
            "<rootDir>/__mocks__/fileTransformer.js"
    },
    testPathIgnorePatterns: ["/node_modules/"],
    testRegex: "(/__tests__/.*|\\.(test|spec))\\.(ts|tsx|js)$",
    modulePaths: ["<rootDir>/src"],
    moduleFileExtensions: ["ts", "tsx", "js", "json"],
    moduleNameMapper: {
        "\\.(css|less)$": "identity-obj-proxy",
        "\\.(jpg|jpeg|png|svg)$": "<rootDir>/__mocks__/fileMock.js"
    },
    collectCoverageFrom: [
        "src/**/*.ts",
        "src/**/*.tsx",
        "!src/**/*.d.ts",
        "!src/**/*.story.tsx",
        "!src/layouts/**/*",
        "!src/types/**/*",
        "!src/scripts/**/*",
        "!src/index.tsx"
    ],
    setupFiles: ["./setupTests.js"],
    roots: ["src"],
    globals: {
        "ts-jest": {
            babelConfig: {
                presets: ["@babel/preset-env", "@babel/preset-react"]
            },
            diagnostics: false
        }
    },
    transformIgnorePatterns: []
}
