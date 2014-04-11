This program converts the text files from the NCDC FEMA snow monitoring program into hdf5 files.

The raw data may be found at http://www.ncdc.noaa.gov/snow-and-ice/dly-data.php and http://www1.ncdc.noaa.gov/pub/data/snowmonitoring/fema/.

The resultant hdf5 file contains two tables:


station table:
- state
- city
- station name
- lat
- lon
- elevation
- coop#
- noaa station id

observation table:
- rows correspond to each station's observations
- first column: my station id number
- subsequent columns: snowfall on a given day (relative to the ...)


Building instructions:
Requires boost, hdf5, C++11 compiler (gcc 4.7+, msvc 2012+, clang), cmake
Run cmake (or ccmake) from another folder pointed at this directory.
After configuring options, run `make'.
