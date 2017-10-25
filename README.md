# rtmath - Radiative Transfer Mathematics Library

## Synopsis

rtmath is a set of radiative transfer-geard routines that are designed to ultimately solve the radiative transfer equation in the microwave spectrum. Furthermore, it contains C++ classes designed to read and manipulate ddscat data and to perform weighting based on particle distributions of shape, size and orientation. These weightings change the bulk scattering and absorption matrices for the DA solution. 

## Building and Installing

DEPENDENCIES:
------------

The core libraries depend on:
- Boost (version >= 1.53)
- A C++ 2011-compatible compiler. This includes any recent versions of gcc/g++, LLVM/clang, or Microsoft Visual C++.
- CMake (generates the build scripts and finds library locations)
- git (used to checkout the code; it provided some information to the build system)
- Doxygen (optional; generates html documentation of the library functions)


The library plugins additionally may depend on:
- tcsh
- postgresql
- HDF5
- netCDF
- LLNL silo
- ceres-solver
- ImageMagick
- voro++
- VTK
- ZLIB
- bzip2

Not all systems provide these dependencies. For CentOS 5.7, for example, everything but cmake must be compiled from scratch. Recent versions of Debian and Ubuntu no longer provide Boost in their repositories, so this must be compiled manually, along with all respective dependencies.

This software is designed to work well on Windows, and may be compiled with GCC, MSVC and Intel compilers. It should work, but is untested, on Mac OS X.


On Debian-based systems (e.g. Ubuntu), the necessary dependencies may be installed using this command:
```
sudo apt install cmake doxygen libhdf5-dev hdf5-tools git zlib1g-dev libnetcdf-dev libboost-all-dev
```
On Fedora, this command may be used:
```
sudo dnf install cmake doxygen hdf5-devel hdf5 git zlib-devel netcdf-devel boost-devel gcc-c++
```
On FreeBSD, use this command:
```
sudo pkg install hdf5 cmake doxygen git netcdf
```
On MacOS, install XCode and Homebrew (https://brew.sh), then run
```
brew install homebrew/science/netcdf doxygen hdf5 cmake git
```

### Notes

- Old CentOS versions (anything below version 6) provide GCC 4.1.x. This lacks C++11 support. A more recent compiler is necessary. GCC can be built for this platform with some difficulty. LLVM is also an option. Also, the version of Boost provided is ancient. This requires a from-source build. Actually, most of the packages probably need a source rebuild.
- For Windows, just install the packages and a compiler (Visual Studio 2010 or better is recommended), and the build works fine.



Building:
-------------

- Download (and perhaps extract) the source code package. 
```
git co https://github.com/rhoneyager/rtmath.git
```

- Create a new build directory. It can be anywhere. Switch into this directory.
```
cd rtmath
mkdir build
cd build
```

- (Optional) Load appropriate modules and build dependencies

- Run cmake to generate the build scripts. e.g.:
```
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX={install path} {Path to source directory}
```
- This command, on Linux, typically defaults to generating Makefiles and will use gcc as the compiler. Consult the CMake
   documentation for details regarding how to change the compiler or other settings.
   To change the compiler, prefix the CMake command with CMAKE_CXX_COMPILER='path to compiler'.

 -- Also, using ccmake or cmake-gui gives a good list of the options that may be set in the build.


Recommended cmake options:

| Option			| Value			|
|-------------------------------|-----------------------|
|CMAKE_INSTALL_PREFIX           | /opt/rtmath		|
|DATA_DIR_PREFIX                | /data/rhoneyag/rtmath |
|ENV_MOD_DIR_PREFIX             | /etc/modulefiles	|
|INSTALL_DATA                   | OFF			|
|INSTALL_MODULES                | ON			|
|USE_IPP                        | OFF			|
|USE_OPENMP                     | OFF			|


- If cmake is set to generate Makefiles, run:
```
make
```
-- If the build is successful, binaries and libraries should be in the ./RelWithDebInfo directory. These can all be copied
to the install directory using:
```
sudo make install
```
--- Note: Installations in nonstandard directories require that the LD_LIBRARY_PATH and PATH environment variables be updated appropriately. On Red Hat-derivatives, 
      the environment-modules package is highly recommended. See documentation for further details.

-- Packages can also be made using 
```
make package
```

## Warning

This library is quite new and under heavy development. Expect much to change, and use at your own risk.

## License

See [LICENSE.txt](./LICENSE.txt)

