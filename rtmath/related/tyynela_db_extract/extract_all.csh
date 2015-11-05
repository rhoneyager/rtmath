#!/bin/tcsh

#./tyynela_db_extract -c ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_cross_G_rosette6_aggregate.nc -p ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_phase_G_rosette6_aggregate.nc -y ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_phys_rosette6_aggregate.nc

set first=1
set ofile=tyynela_combined_agg.tsv
# no g-band graupel
foreach band ( G W Ka Ku X C S )
foreach atype ( rosette6_aggregate needle_aggregate ferndend_aggregate steldend_aggregate )

if ( $band == "G" ) then
if ( $atype == "graupel") then
echo Skipping g-band graupel
continue

endif
endif

echo $band $atype
# Redo nc file as hdf5
#nccopy -k hdf5 ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_cross_${band}_${atype}.nc h5/Tyynela_240614_cross_${band}_${atype}.hdf5
#nccopy -k hdf5 ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_phase_${band}_${atype}.nc h5/Tyynela_240614_phase_${band}_${atype}.hdf5
#nccopy -k hdf5 ~/Tyynela/Tyynela/helios.fmi.fi/tyynelaj/database/Tyynela_240614_phys_${atype}.nc h5/Tyynela_240614_phys_${atype}.hdf5

./tyynela_db_extract -c h5/Tyynela_240614_cross_${band}_${atype}.hdf5 -p h5/Tyynela_240614_phase_${band}_${atype}.hdf5 -y h5/Tyynela_240614_phys_${atype}.hdf5 -o out-${band}-${atype}.tsv

if ( $first ) then
	cat out-${band}-${atype}.tsv > $ofile
else
	tail -n +2 out-${band}-${atype}.tsv >> $ofile
endif
set first=0

end
end

