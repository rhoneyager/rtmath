process_file is a program for reading a directory of netcdf files and extracting the 
relevent information in each file to populate arm database entries. This processor provides 
a first-pass of the files. It extracts the site, instrument, start and end times of the 
measurements. It then connects to the postgresql database (using the settings in ~/.rtmath) 
and adds the appropriate entries into the database.

When adding the entries, uuid generation is necessary. It does this by using the boost uuid 
classes. It also verifies that the file entry is unique in the database.

This program does _not_ verify that clouds are overhead. That is a special type of processing 
that involves detailed knowledge of the different instrument file formats. The instruments 
that are used for cloud detection are the vaisala celiometer, total sky imager, multiple 
lidars, interferometers and radar. However, instruments with a small file size are preferred 
because the cloud existence data is preserved for future radar retreival selection.
