# /usr/bin/python
#
#==--- MaterialDoxygen/FetchFiles.sh ----------------------------------------==#
#
# File        : FetchFiles.sh
# Description : Fetches the files required for formatting Voxel's Doxygen
#               documentation.
#
#==--------------------------------------------------------------------------==#

import subprocess, os

def git(*args):
  return subprocess.check_call(['git'] + list(args))

baseUrl =\
  'https://raw.githubusercontent.com/Voxelated/MaterialDoxygen/master/'

urls = {
  'stylesheet'  : 'css/stylesheet.css',
  'header'      : 'html/header.html'  ,
  'footer'      : 'html/footer.html'  ,
  'navbar'      : 'html/navbar.html'  ,
  'jscript'     : 'js/DoxyFormat.js'  ,
  'logo'        : 'images/VoxelLogoSmall.png'
}

os.mkdir('DoxyFormat')
os.chdir('DoxyFormat')

for key, val in urls.items():
  subprocess.call(['wget', '--no-cache', baseUrl + val])
