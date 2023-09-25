mkdir thirdparty
cd thirdparty
git clone https://github.com/rmsalinas/DBow3.git
cd DBow3
mkdir build
cd build
cmake ..
make -j4
maek install
