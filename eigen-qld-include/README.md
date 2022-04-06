# Question
Is it possible to remove the following line from ProjectB/CMakeLists.txt?

```
find_package(eigen-qld REQUIRED)
```

# Build
```
TOPDIR=`pwd`
rm -rf install

cd ${TOPDIR}/ProjectA
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${TOPDIR}/install
make
make install

cd ${TOPDIR}/ProjectB
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=${TOPDIR}/install
make
make install
```
