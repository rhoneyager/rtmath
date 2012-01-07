rtmath
Ryan's Radiative Transfer (and Other Stuff) Library
---------------

The files in and under this directory are the complete source distribution of the rtmath library. This library is intended to eventually cover everything concerning radiative transfer in an arbitrary atmosphere, from the actual radiative transfer equation solutions to the calculation of ensemble scattering phase functions. It also contains many supplementary classes handling tasks such as recursive polynomial generation, zero-finding and matrix mathematics. 

rtmath source is intended to be compilable using the a recent version of the GNU compiler collection (more specifically, g++ with support for the C++ 2012 standard) or Microsoft Visual Studio 2010 or later. 

The files are split into several sections for ease of installation. Headers are located under the headers/ directory. Source code is under src/, and any binaries produced (including test binaries, DLLs, SOs and the like will be located under bin/ and lib/).

A makefile is included for sustems supporting GNU make. To make the library, just run 'make'. Tests may be conducted with 'make test', and the library may be installed to system folders with 'make install'. To remove object code and binaries, run 'make clean'.
