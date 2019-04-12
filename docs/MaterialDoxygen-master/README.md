# MaterialDoxygen

Formatting files for Voxel's doxygen documentation generation. Using the
additional files provided here as the extra files which can be supplied to
Doxygen will produce documentation which is based on Google's material design.

The stylesheets which are loaded are as follows:

1. __Doxygen__: Each of the html files first loads the doxygen stylesheet.

2. __Host site__: The __header.html__ file then tries to load a stylesheet for
                  the host site, which is assumed to be at __/css/main.css__.
                  This is to allow the documentation to look the same as the
                  host site.

3. __Custom__: Lastly, a custom stylesheet is loaded, which is specified in the
              **HTML_EXTRA_STYLESHEET** parameter for doxygen.

The default __stylesheet.css__ is generated from the __scss__ files in the
[Voxel documentation respository](https://github.com/Voxelated/voxelated.github.io) -- see the **_sass** directory, and the README for how to
compile the sass files into a css file. By changing the variables in the
sass files, and replacing the **stylesheet.css** file in this repository with
the compiled modified scss files, custom looking Doxygen based on Google's
material theme can be produced.

A live demo of the look of documentation generated using these modifications can
be found at: [Voxel docs](https://voxelated.github.io/libraries/voxel/index.html)

