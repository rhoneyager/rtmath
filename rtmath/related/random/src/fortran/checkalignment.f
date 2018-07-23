! This mini-module allows fortran code to throw some errors 
! back up to C++
      module throwers
      interface
      subroutine throw_int(i) bind(C, name="throw_int")
      use iso_c_binding
      implicit none
      integer (C_INT) :: i
      end subroutine throw_int
      end interface

      end module throwers


! This program provides a function that checks the proper
! alignment of input parameters. It is intended for use in
! the testing portion of the library code. If this check fails,
! then running the code is pointless.

      subroutine check_aligns(x) BIND(C, NAME='check_aligns_')

      use, intrinsic :: ISO_C_BINDING
!      use throwers
      IMPLICIT REAL*8 (A-H,O-Z)
#include "common.defs"
      real*8 :: tolerance = 0.00001
      INTEGER (C_INT) :: x
! Now, check alignment based on default params in tmatrix.cpp
! Good alignment returns true, bad alignment returns false

      x = 0
      if ( (axi - 10) .gt. tolerance) x = x + 1
      if (rat - 0.1 .gt. tolerance) x = x + 2
      if (lam - (DACOS(-1D0)*2D0) .gt. tolerance) x = x + 4
      if (mrr - 1.5 .gt. tolerance) x = x + 8
      if (mri - 0.02 .gt. tolerance) x = x + 16
      if (eps - 0.5 .gt. tolerance) x = x + 32
      if (DDELT - 0.001 .gt. tolerance) x = x + 64
      if (alpha - 145 .gt. tolerance) x = x + 128
      if (beta - 52 .gt. tolerance) x = x + 256
      if (thet0 - 56 .gt. tolerance) x = x + 512
      if (thet - 65 .gt. tolerance) x = x + 1024
      if (phi0 - 114 .gt. tolerance) x = x + 2048
      if (phi - 128 .gt. tolerance) x = x + 4096

      if (np - (-1) .gt. tolerance) x = x + 8192
      if (ndgs - 2 .gt. tolerance) x = x + 16384

!      throw_int(1)

      return
      end

