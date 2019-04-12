var gulp = require('gulp');
var sourcemaps = require('gulp-sourcemaps');
var source = require('vinyl-source-stream');
var babel = require('babelify');
var gutil = require('gulp-util');
var gexit = require('gulp-exit');
var tsify = require("tsify");
var ts = require('gulp-typescript');
var browserify = require('browserify');
var watchify = require('watchify');

// tsProject over all functions
let tsProject = ts.createProject('tsconfig.json');

gulp.task('default', () => {return defaultBuild()});
gulp.task('defaultBuild', ['tsbuild']);
gulp.task('b', ['browserify']);
gulp.task('watch', () => {return tsBuild(true)});
/**
 * Builds the ts project with tsconfig.json and outputs to build
 */
 let srcFiles = [
   '**/*.tsx',
   '**/*.ts',
   '!node_modules/**',
   '!test/**',
   '!build/**'
 ];

gulp.task('tsbuild', function(){
  // get the project source and pipe it to gulp-typescript
  // include sourcemaps when react-native doesn't suckss
  // https://github.com/ivogabe/gulp-typescript#source-maps
  // let tsResult = tsProject.src()
  let tsResult = gulp.src(srcFiles, { base: './app/src' })
    .pipe(sourcemaps.init()) // generate sourcemaps
    .pipe(ts(tsProject)).js
    .pipe(sourcemaps.write('.')) // append sourcemap to each file
    .pipe(gulp.dest('app/build'));
  return tsResult;
});

gulp.task('browserify', function() {
    var b = browserify({
        entries: ['./app/build/app.js'], // Only need initial file, browserify finds the deps
        transform: [babel], // We want to convert JSX to normal javascript
        debug: true, // Gives us sourcemapping
        cache: {}, packageCache: {}, fullPaths: true // Requirement of watchify
    });
    return b.bundle() // Create the initial bundle when starting the task
    .pipe(source('bundle.js'))
    .pipe(gulp.dest('./app/build/'));
});

/**
 * Moves required html into build
 */
 // gulp.task('moveHtml', function(){
 //   let htmlSrcs = [
 //     'html/index.html',
 //     'html/images/**'
 //   ];
 //   return gulp.src(htmlSrcs, { base: './' })
 //     .pipe(gulp.dest('build'));
 // });
