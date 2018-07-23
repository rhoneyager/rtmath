      program T3
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .................................................................
c     .  calculate the differential scattering cross section in all   . 
c     .    directions by projecting the surface of a spherical        .
c     .    surface onto a rectangular coordinate system.              .
c     .    thsc: the theta scattering angle, is projected onto the    .
c     .          radius r in the rectangular plane                    .
c     .          r = thsc/180 degrees   (r varies from 0 to 1)        .
c     .          when r > 1, set scattering to backscatter value      .
c     .    phsc: the phi scattering angle, is the azimuthal angle in  .
c     .          the spherical and rectangular coordinate systems     .
c     .                                                               .
c     .                               |                               .
c     .                      x     x  |  x      x                     .
c     .                               |                               .
c     .                      x     x  |  x      x                     .
c     .                    ------------------------ y                 .
c     .                      x     x  |  x      x                     .
c     .                               |                               .
c     .                      x     x  |  x      x                     .
c     .                               |                               .
c     .                               x                               .
c     .                                                               .
c     .  start at (x,y) = (-1,-1) and continue in the + x direction   .
c     .    then increment y by dlt and continue at (-1,-1+dlt)        .
c     .                                                               .
c     .  inputs:  angint = theta orientation in the laboratory frame  .
c     .           anginp = phi orientation in the laboratory frame    .
c     .           anglab = angle of the E-field polarization vector   .
c     .                    in the x-y plane                           .
c     .           npnts  = number of grid points = npnts x npnts      .
c     .................................................................
      complex tmat,acans
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25,26) 
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,ic,thsc 
      common /outcom/ total
      dimension cmx(25),ac(1000)
c     ...........................
c     .  set program constants  .
c     ...........................
      dtr = .017453292519943 
      rtd = 57.2957795131 
      pi = 3.14159265358979 
c     ....................................................
c     .  read from file:                                 .
c     .        the T-matrix and normalization constants  .
c     .        nrank = number of terms (matrix order)    .
c     .        nm    = number of azimuthal modes m       .
c     .        x     = size parameter (ka)               .
c     ....................................................
      open(unit=11,file='t',form='unformatted')
      rewind 11
      open(unit=12,file='case')
      rewind 12
      read(12,100) nm,nrank,x
      close(unit=12)
      nranki = nrank+1
      write(6,110)
      write(6,*) 'enter angint,anginp,anglab,npnts'
      read(5,*) angint,anginp,anglab,npnts
c     ...............................................................
c     .  calculation for angint = 0 or 180 degrees uses m = 1 only  .
c     .    set the do 30 and do 50 loop indices for m = 1 only      .
c     .    read the stored arrays once to skip over the T-matrix    .
c     .    and normalization constants for m = 0                    .
c     ...............................................................
      if(angint.eq.0.0.or.angint.eq.180.0) then
        nl = 2
        nh = 2
        ic = 1
        read(11) tmat,cmx
c     ...........................................................
c     .  calculation for angint not 0 or 180 degrees ( all m )  .
c     .    set the do 30 and do 50 loop indices for all m       .
c     ...........................................................
      else
        nl = 1
        nh = nm
        ic = 2
      end if
      angint = angint*dtr
      anginp = anginp*dtr
      anglab = anglab*dtr
      tp = anginp
c     .................................................................
c     .  utheta and uphi are the amplitudes of the incident field in  .
c     .    the theta (parallel) and phi (perpendicular) directions    .
c     .................................................................
      utheta = cos(anglab)
      uphi = sin(anglab)
c     .............................................
c     .  calculate the scattered field expansion  .
c     .    coefficients and the backscatter       .
c     .  enter a loop for each azimuthal mode m   .
c     .............................................
      do 30 im = nl,nh
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
        kmv = im-1
        cmv = real(kmv)
        prodm = 1.0
        if(kmv.gt.0) then
          quanm = cmv
          do 10 i = 1,kmv
            quanm = quanm+1.0
            prodm = quanm*prodm/2.0
10        continue
        end if
        twm = 2.0*cmv
        ij = kmv-1
        if(ij.lt.0) ij = 0
        ns = nrank-ij
c     ..............................................................
c     .  read the T-matrix and normalization constants for each m  .
c     ..............................................................
        read(11) tmat,cmx
        do 20 k = 1,ns
          cmxnrm(k,im) = cmx(k)
20      continue
c     ..........................................................
c     .  calculate the scattered field expansion coefficients  .
c     .    and sum the backscatter over all azimuthal modes    .
c     ..........................................................
        call addprc
30    continue
      close(unit=11)
c     ......................................................
c     .  calculate the logarithm of the total backscatter  .
c     .    differential scattering cross section           .
c     ......................................................
      bkscat = alog10(total)
      open(unit=9,file='t3.dat')
      rewind 9
c     .....................................................
c     .  calculate the differential scattering cross      .
c     .    section over a rectangular grid corresponding  .
c     .    to the surface of a sphere                     .
c     .....................................................
      dlt = 2.0/real(npnts-1)
c     ....................................
c     .  set starting y grid value (yg)  .
c     .  enter loop to vary y values     .
c     ....................................
      yg = -1.0
      do 70 ig = 1,npnts
c     ....................................
c     .  set starting x grid value (xg)  .
c     .  enter loop to vary x values     .
c     ....................................
        xg = -1.0
        do 60 jg = 1,npnts
          r = sqrt(xg**2+yg**2)
          if(r.lt.1.0) then
c     .............................................................
c     .  calculate the theta and phi scattering angles for radii  .
c     .    within the unit circle on the rectangular grid         .
c     .............................................................
            thsc = r*pi
            phsc = 0.0
            if(abs(xg).gt.0.0.or.abs(yg).gt.0.0) phsc = atan2(yg,xg)
c     ...............................................................
c     .  rotate the phi orientation angle anginp with phsc so that  .
c     .    the scattering at the angle phsc can be obtained in      .
c     .    the phi = zero degree plane in the laboratory frame      .
c     ...............................................................
            anginp = tp-phsc
            if(anginp.lt.0.0) anginp = anginp+2.0*pi
c     ............................................
c     .  enter a loop for each azimuthal mode m  .
c     ............................................
            do 50 im = nl,nh 
c     ...............................
c     .  set m-dependent constants  .
c     ...............................
              kmv = im-1
              cmv = real(kmv)
              prodm = 1.0
              if(kmv.gt.0) then
                quanm = cmv
                do 40 i = 1,kmv
                  quanm = quanm+1.0
                  prodm = quanm*prodm/2.0
40              continue
              end if
              twm = 2.0*cmv
c     ..............................................................
c     .  calculate the differential scattering cross section at    .
c     .    each grid point - prcess is an alternate entry to       .
c     .    subroutine addprc for scattered field calculations      .
c     .    using the previously calculated expansion coefficients  .
c     ..............................................................
              call prcess 
50          continue 
c     .............................................
c     .  calculate the logarithm of the total     .
c     .    differential scattering cross section  .
c     .    at (xg,yg) and store in array ac(*)    .
c     .............................................
            ac(jg) = alog10(total)
          else
c     ....................................................
c     .  set the differential scattering cross section   .
c     .    to the backscatter value for all grid points  .
c     .    on or outside the unit circle                 .
c     ....................................................
            ac(jg) = bkscat
          end if
c     .......................
c     .  increment x value  .
c     .......................
          xg = xg+dlt
60      continue
c     ......................................
c     .  write out data for all xg values  .
c     .    (for given yg value)            .
c     ......................................
        write(9,120) (ac(j),j=1,npnts)
c     .......................
c     .  increment y value  .
c     .......................
        yg = yg+dlt
70    continue
      close(unit=9)
      stop
100   format(2i4,f8.4)
110   format('.....................................................',/,
     1       '.  calculate scattered intensity in all directions  .',/,
     2       '.  output is written to t3.dat                      .',/,
     3       '.....................................................',//,
     4       'angint: theta orientation in the lab frame, degrees',/,
     5       'anginp: phi orientation in the lab frame, degrees'/,
     6       'anglab: angle of the E-field polarization',/,
     7       '        vector in the x-y plane, degrees',/, 
     8       'npnts: number of grid points (npnts x npnts)',/)
120   format(e14.6)
      end
      subroutine genlgp(pnmllg,nc)
c     ........................................................
c     .  calculate associated Legendre functions (argument   .
c     .    cos(theta)) divided by sin(theta) for each        .
c     .    azimuthal mode m                                  .
c     .  generate first two orders by formula and remaining  .
c     .    orders by recursion                               .
c     .                                                      .
c     .  pnmllg = associated Legendre function/sin(theta)    .
c     .  sinth = sin(theta), costh = cos(theta)              .
c     .  nc = number of orders (0 to nc-1)                   .
c     .  the order of the associated Legendre functions is   .
c     .    incremented by one in the pnmllg(*) array         .
c     ........................................................
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      dimension pnmllg(nc)
      dtwm = twm+1.0 
c     ................................................................
c     .  calculate for the special case theta = 0 degrees or 180     .
c     .    degrees - note that for theta = 180 degrees, costh = -1,  .
c     .    but theta is set to 0 degrees prior to subroutine entry   .
c     ................................................................
c     ............................................................
c     .  if theta = 0 degrees and m.ne.1 all functions are zero  .
c     ............................................................
      if(theta.eq.0.and.kmv.ne.1) then
        do 10 ilg = 1,nc
          pnmllg(ilg) = 0.0 
10      continue 
        return
      end if
c     ............................................................
c     .  if theta = 0 degrees and m.eq.1 calculate orders 0,1,2  .
c     ............................................................
      if(theta.eq.0.and.kmv.eq.1) then
        pnmllg(1) = 0.0 
        pnmllg(2) = 1.0 
        pla = 1.0 
        plb = 3.0*costh
        pnmllg(3) = plb
        ibeg = 4
c     ...............................................................
c     .  if theta.ne.0 degrees and m.eq.0 calculate orders 0 and 1  .
c     ...............................................................
      else if(kmv.eq.0) then
        pla = 1.0/sinth
        plb = costh*pla 
        pnmllg(1) = pla 
        pnmllg(2) = plb 
        ibeg = 3 
c     ..........................................................
c     .  if theta.ne.0 degrees and m.ne.0 calculate first two  .
c     .    nonzero orders - the associated Legendre function   .
c     .    is zero for orders less than the azimuthal mode m   .
c     ..........................................................
      else
        do 20 ilg = 1,kmv 
          pnmllg(ilg) = 0.0 
20      continue 
        pla = prodm*sinth**(kmv-1) 
        pnmllg(kmv+1) = pla 
        plb = dtwm*costh*pla 
        pnmllg(kmv+2) = plb 
        ibeg = kmv+3 
      end if
c     .................................................
c     .  recur upward to obtain all remaining orders  .
c     .................................................
      if(ibeg.gt.nc) return
      cnmul = real(2*ibeg-3) 
      cnm = 2.0 
      cnmm = dtwm 
      do 30 ilgr = ibeg,nc 
        plc = (cnmul*costh*plb-cnmm*pla)/cnm                            eq 3.76
        pnmllg(ilgr) = plc 
        pla = plb 
        plb = plc 
        cnmul = cnmul+2.0 
        cnm = cnm+1.0 
        cnmm = cnmm+1.0 
30    continue 
      return 
      end 
      subroutine genang(n,tang,pang,cosb,sinb) 
c     ...............................................................
c     .  calculate scattering angles in the particle frame corres-  .
c     .    ponding to scattering angles in the laboratory frame     .
c     .  calculate the elements of the transform matrix for the     .
c     .    polarization vectors                                     .
c     ...............................................................
      common dtr,rtd,pi 
      common /uvccom/ angint,anginp,utheta,uphi,ic,thsc
      sint = sin(angint)
      cost = cos(angint)
      sinp = sin(anginp)
      cosp = cos(anginp)
c     ...............................................................
c     .  calculate scattering angles in the particle frame corres-  .
c     .    ponding to scattering angles in the laboratory frame     .
c     ...............................................................
      ps = 0.0
      sints = sin(thsc)
      costs = cos(thsc)
      cosps = cos(ps)
      x = -cosp*cost*sints*cosps+sint*costs                             eq 3.29a
      y = sinp*sints*cosps                                              eq 3.29b
      z = cosp*sint*sints*cosps+cost*costs                              eq 3.29c
c     .................................................................
c     .  calculate the theta scattering angles in the particle frame  .
c     .................................................................
      r = sqrt(x**2+y**2)
      tang = 0.0
      if(z.ne.0.0.or.r.ne.0.0) tang = atan2(r,z)                        eq 3.30a
c     ...............................................................
c     .  calculate the phi scattering angles in the particle frame  .
c     ...............................................................
      pang = 0.0
      if(x.ne.0.0.or.y.ne.0.0) pang = atan2(y,x)                        eq 3.30b
      if(pang.lt.0.0) pang = 2.0*pi+pang
c     ....................................................
c     .  calculate the elements of the transform matrix  .
c     .    matrix for the polarization vectors           .
c     .  the transform matrix is needed only for the     .
c     .    incident polarization vector ( case n = 1 )   .
c     ....................................................
      if(n.eq.1) then
        cosb = -cosp                                                    eq 3.40a
        sinb = sinp                                                     eq 3.40b
      end if
      return 
      end 
      subroutine addprc 
c     ...............................................................
c     .  calculate the scattered field expansion coefficients and   .
c     .    the differential scattering cross section at angles      .
c     .    tang, pang in the particle frame corresponding to        .
c     .    angles thsc, phsc in the laboratory frame                .
c     .  entry at addprc calculates the incident field expansion    .
c     .    coefficients and scattered field expansion coefficients  .
c     .    which only need to be calculated once for each m         .
c     .  entry at prcess uses the previously calculated scattered   .
c     .    field expansion coefficients to calculate the            .
c     .    differential scattering cross section                    .
c     .  calculations are performed in the particle frame           .
c     ............................................................... 
      complex tmat,ab1(50),ab2(50),ci,c1,c2,cim,acans,
     1fg1(50,26),fg2(50,26) 
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25,26) 
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x 
      common /uvccom/ angint,anginp,utheta,uphi,ic,thsc 
      common /outcom/ total
      dimension pnmllg(26),acans(2)
      logical first
      data first /.true./
c     ...........................................................
c     .  on first entry set the constants and:                  .
c     .    (1) calculate the incident field direction thtinc    .
c     .          in the particle frame                          .
c     .    (2) transform the incident field polarization        .
c     .          vector from the laboratory frame to the        .
c     .          particle frame                                 .
c     .          uu1 = theta component and uu2 = phi component  .
c     .    (3) set the backscatter angle thsc to 180 degrees    .
c     ...........................................................
      if(first) then
        ci = (0.0,1.0)
        nr2 = 2*nrank
        snorm = 1.0/(pi*x**2)
        thsc = 0.0
        call genang(1,tang,pang,cosb,sinb)
        thtinc = tang
        uu1 = cosb*utheta-sinb*uphi                                     eq 3.38a
        uu2 = sinb*utheta+cosb*uphi                                     eq 3.38b
        thsc = pi
        first = .false.
      end if
c     ...................................................
c     .  set indices for matrix compression when n < m  .
c     .    note: useful only when m > 1                 .
c     ...................................................
      ij = kmv-1
      if(ij.lt.0) ij = 0
      ijt = 2*ij
      ns = nrank-ij
      ns2 = 2*ns
c     .........................................................
c     .  calculate the associated Legendre functions for the  .
c     .    incident angle = thtinc in the particle frame      .
c     .........................................................
      theta = thtinc 
      sinth = sin(theta)
      costh = cos(theta)
      call genlgp(pnmllg,nranki) 
c     .................................................
c     .  calculate the incident field coefficients:   .
c     .    ab1(*) = theta polarization                .
c     .    ab2(*) = phi polarization                  .
c     .  note: ab1(*) and ab2(*) used in scattered     .
c     .        field programs are j times ab1(*) and   .
c     .        ab2(*) used in internal field programs  .            
c     .................................................
      do 10 n = 1,nrank 
        if(n.le.ij) go to 10
        np = n+nrank 
        cn = real(n)
        n1 = n+1 
        c1 = 4.0*ci**n 
        c2 = 4.0*ci**n1 
        p1 = cn*costh*pnmllg(n1)-(cn+cmv)*pnmllg(n) 
        p2 = cmv*pnmllg(n1) 
        ab1(n-ij) = c1*p2*uu1 
        ab1(np-ijt) = -c2*p1*uu1 
        ab2(n-ij) = -c1*p1*uu2 
        ab2(np-ijt) = c2*p2*uu2 
10    continue 
c     ....................................................
c     .  the scattered field coefficients = -[T] times   .
c     .    the incident field coefficients               .
c     .  store the coefficients for all azimuthal modes  .
c     .  fg2(ns+1) to fg2(ns2) is -g(omn)                .
c     ....................................................
      do 30 i = 1,ns2 
        fg1(i,kmv+1) = 0.0
        fg2(i,kmv+1) = 0.0
        do 20 j = 1,ns2
          fg1(i,kmv+1) = fg1(i,kmv+1)-tmat(i,j)*ab1(j)                  eq 3.12a
          fg2(i,kmv+1) = fg2(i,kmv+1)-tmat(i,j)*ab2(j)                  eq 3.12b
20      continue 
30    continue 
c     ..............................................................
c     .  alternate entry to calculate the differential scattering  .
c     .    cross section using the previously calculated           .
c     .    scattered field expansion coefficients                  .
c     ..............................................................
      entry prcess
c     ................................................................
c     .  clear the result accumulators and calculate the coordinate  .
c     .    transformation variables for each new grid point          .
c     ................................................................ 
      if(ic.eq.1.or.kmv.eq.0) then
        acans(1) = 0.0
        acans(2) = 0.0
        call genang(2,tang,pang,cosb,sinb)
      end if
c     ...................................................
c     .  set indices for matrix compression when n < m  .
c     .    note: useful only when m > 1                 .
c     ...................................................
      ij = kmv-1
      if(ij.lt.0) ij = 0
      ijt = 2*ij
c     ...........................................................
c     .  evaluate the scattered field at each scattering angle  .
c     .    theta = tang and phi = pang in the particle frame    .
c     .  results are accumulated in acans(*) for each           .
c     .    azimuthal mode m                                     .
c     .  acans = kF, where F is a component of the vector       .
c     .    far-field amplitude                                  .
c     ...........................................................
c     .................................................
c     .  calculate the associated Legendre functions  .
c     .    at each scattering angle                   .
c     .................................................
      theta = tang
      sinth = sin(theta)
      costh = cos(theta)
      if(tang.eq.pi) then
        theta = 0.0
        costh = -1.0
      end if
      call genlgp(pnmllg,nranki) 
      phi = cmv*pang
      sinphi = sin(phi)
      cosphi = cos(phi)
      do 40 n = 1,nrank
        if(n.le.ij) go to 40
        np = n+nrank 
        n1 = n+1 
        cn = real(n)
        p1 = cn*costh*pnmllg(n1)-(cn+cmv)*pnmllg(n) 
        p2 = cmv*pnmllg(n1) 
        aa = sinphi*p1 
        bb = cosphi*p1 
        cc = sinphi*p2 
        dd = cosphi*p2 
        cim = ci**(-n1) 
        nij = n-ij
        npijt = np-ijt
c     .............................................
c     .  solve for the theta polarized scattered  .
c     .    field in the particle frame            .
c     .............................................
      acans(1) = acans(1)+cim*(fg1(nij,kmv+1)*dd+ci*fg1(npijt,kmv+1)*bb eq 3.17a
     1  -fg2(nij,kmv+1)*cc-ci*fg2(npijt,kmv+1)*aa)*cmxnrm(nij,kmv+1)    eq 3.18a
c     ...........................................
c     .  solve for the phi polarized scattered  .
c     .    field in the particle frame          .
c     .  fg2(npijt) is -g(omn) per (3.12b)      .
c     ...........................................
      acans(2) = acans(2)-cim*(fg1(nij,kmv+1)*aa+ci*fg1(npijt,kmv+1)*cc eq 3.17b
     1  +fg2(nij,kmv+1)*bb+ci*fg2(npijt,kmv+1)*dd)*cmxnrm(nij,kmv+1)    eq 3.18b
40    continue 
c     ..................................................................
c     .  normalize the converged results and calculate the total       .
c     .    differential scattering cross section (both polarizations)  .  
c     .    in the particle frame (same as in the laboratory frame)     .
c     ..................................................................
      if(ic.eq.1.or.kmv.eq.(nm-1)) then
        total = snorm*(abs(acans(1))**2+abs(acans(2))**2)
      end if
      return
      end
