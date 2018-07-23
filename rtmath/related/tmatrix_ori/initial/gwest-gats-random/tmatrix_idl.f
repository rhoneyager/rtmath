
c--------------------------------------------------------------------------------
c
c This routine provides a simple interface to IDL.  
c
c The calling IDL routine ("tmatrix_idl_call.pro") writes the user input parameters 
c to an ascii file, which this routine reads and uses. 
c
c This routine runs the Tmatrix fortran code and then writes the results to another 
c ascii file which the IDL routine reads and returns.
c
c Input (from IDL routine):
c   wave = wavelength (microns)
c   mrr  = real index
c   mri  = imaginary index, 
c   rad  = radius (microns)
c   ar   = spheriod axial ratio
c
c- to complie, type: gfortran -o tmatrix_idl tmatrix_idl.f tmd.lp_sub.f lpd.f
c- to run: .//tmatrix
c
c Source: Mark Hervig
c--------------------------------------------------------------------------------
      
      IMPLICIT REAL*8 (A-H,O-Z)            
      real*8 wave,mrr,mri,rad,ar   
      character*100 fileo,filei
      
      filei = '/users/mhervig/fortlib/t-matrix/tmatrix_idl_in.txt'    ! the input file
      fileo = '/users/mhervig/fortlib/t-matrix/tmatrix_idl_out.txt'   ! the output
      
c-    get the input from the IDL routine
            
      open(4,file=filei,form='formatted',status='old')

      read(4,*) wave,mrr,mri,rad,ar
        
      close(4)
            
c-    open the output file
      
      open(5,file=fileo)
                  
c-    call the Goddard T-matrix routine
		
      NPNAX  = 1        ! the next 4 lines setup monodispersed result (single particle)                              
      B      = 1D-1   
      NKMAX  = -1                                                      
      NDISTR = 4 
      	   
      GAM    = 0.5D0      
      RAT    = 1D0      ! 1 = equal-volume-sphere radii are used
      NP     = -1       ! -1 = spheroids             
      DDELT  = 0.001D0  ! accuracy
      NPNA   = 19       ! # scattering angles
      NDGS   = 7        ! For compact particles, recommended value is 2, For highly aspherical particles larger values (3, 4,...) may be necessary

      call tmatrix_sub(rat,ndistr,rad,npnax,b,gam,nkmax,
     &                 ar,np,wave,mrr,mri,ddelt,npna,ndgs,
     &                 ext_xsec,sca_xsec)  
	
      write (5,'(2e14.6)') ext_xsec,sca_xsec  ! extinction cross section in um^2  
      
      close(5)
           
c-    done

      stop
      end