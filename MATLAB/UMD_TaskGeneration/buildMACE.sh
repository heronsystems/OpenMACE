cd $MACE_ROOT
rm -rf build
rm -rf lib/*
rm -rf include/*
mkdir build
cd build
qmake ../src/src.pro
make
make install
