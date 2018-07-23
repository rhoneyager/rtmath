      program T8
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .........................................................
c     .  calculate the internal intensity on a central line   .
c     .    or a rectangular grid for a spheroid in the        .
c     .    equatorial (x-y) plane                             .
c     .  inputs: ip = polarization parallel (1)               .
c     .               or perpendicular (2) to the x-y plane   .
c     .          npnts = number of points (multiple of 4)     .
c     .          idim = graph dimension (2 or 3)              .
c     .                                                       .
c     .  calculations are made only for grid points lying on  .
c     .    or inside an inscribed sphere within the spheroid  .
c     .........................................................
      complex a,cd1,cd2,s(3),ci,bslcmp(26),cb,sb,bc,b,rc,cmx,cm
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,a(50,50)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ cd1(50,26),cd2(50,26),p(26,26)
      dimension ac(1000),phi(2)
c     ...........................
c     .  set program constants  .
c     ...........................
      dtr = .017453292519943 
      rtd = 57.2957795131 
      pi = 3.14159265358979 
      ci = (0.0,1.0)
c     ..................................................
c     .  read from file:                               .
c     .        the [A] matrix                          .
c     .        nrank = number of terms (matrix order)  .
c     .        nm    = number of azimuthal modes m     .
c     .        x     = size parameter (ka)             .
c     .        aovrb = axial ratio (a/b)               .
c     .        cmr,cmi = complex index of refraction   .
c     ..................................................
      open(unit=11,file='a',form='unformatted')
      rewind 11
      open(unit=12,file='case')
      rewind 12
      read(12,200) nm,nrank,x,aovrb,cmr,cmi
      cm = cmplx(cmr,cmi)
      close(unit=12)
      nranki = nrank+1
      write(6,210)
      write(6,*) 'enter polarization, npnts, idim'
      read(5,*) ip,npnts,idim
      open(unit=9,file='t8.dat')
      rewind 9
c     ..................................
c     .  set npnts to a multiple of 4  .
c     ..................................
      npnts = 4*(npnts/4)
      npnt = npnts/2
      npm = npnts+1
      if(idim.eq.2) then
        ngm = 2
        write(6,220) npnts
      else
        ngm = npnts/4+1
        write(6,230) ngm,npnts
        do 10 i = 1,npnts
          ac(i) = 0.0
10      continue
        write(9,250) (ac(j),j = 1,npnts)
      end if
      dltx = 2.0/real(npnts-1)
      dlty = 2.0/real(npnt)
c     .........................................................
c     .  calculate the internal field expansion coefficients  .
c     .........................................................
c     .............................................
c     .  enter a loop for each azimuthal mode m   .
c     .............................................
      do 30 im = 1,nm
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
        kmv = im-1
        cmv = real(kmv)
        prodm = 1.0
        if(kmv.gt.0) then
          quanm = cmv
          do 20 i = 1,kmv
            quanm = quanm+1.0
            prodm = quanm*prodm/2.0
20        continue
        end if
        twm = 2.0*cmv
        ij = kmv-1
        if(ij.lt.0) ij = 0
        ns = nrank-ij
c     ....................................
c     .  read the [A] matrix for each m  .
c     ....................................
        read(11) a
c     .........................................................
c     .  calculate the internal field expansion coefficients  .
c     .........................................................
        call addprc
30    continue
      close(unit=11)
c     ........................................................
c     .  set radius of inscribed sphere within the spheroid  .
c     ........................................................
      risc = min(x,x/aovrb)
c     ....................................
c     .  set starting y grid value (yg)  .
c     .  enter loop to vary y values     .
c     ....................................
      yg = real(2-idim)*(1.0-dlty)
      do 110 ig = 2,ngm
c     .....................................................
c     .  set starting x grid value (xg)                   .
c     .  ns = first grid point on or inside radius risc   .
c     .  enter loop to vary x values                      .
c     .....................................................
        ns = int((1.0-sqrt(1.0-yg**2))/dltx)+2
        if(ig.eq.ngm) ns = 1
        xg = -1.0+dltx*real(ns-1)
        do 90 jg = ns,npnt
          ra = sqrt(xg**2+yg**2)
          cmx = ra*cm*risc
          call besj(cmx,bslcmp,nranki)
          pa = atan(yg/xg)
          phi(1) = pi+pa
          phi(2) = 2.0*pi-pa
c     ...........................................
c     .  phi(1) calculates intensity for x < 0  .
c     .  phi(2) calculates intensity for x > 0  .
c     ...........................................
          do 80 iphi = 1,2
            do 50 k = 1,3
              s(k) = 0.0
50          continue
            do 70 im = 1,nm
              kmv = im-1
              cmv = real(kmv)
              ij = kmv-1
              if(ij.lt.0) ij = 0
              ijt = 2*ij
              sn = sin(cmv*phi(iphi))
              cs = cos(cmv*phi(iphi))
              do 60 n = 1,nrank
                if(n.gt.ij) then
                  np = n+nrank
                  n1 = n+1
                  cn = real(n)
                  p1 = -(cn+cmv)*p(n,im)
                  p2 = cmv*p(n1,im)
                  cb = cs*bslcmp(n1)
                  sb = sn*bslcmp(n1)
                  bc = bslcmp(n1)/cmx
                  b = bslcmp(n)-cn*bc
                  rc = cn*(cn+1.)*p(n1,im)*bc
                  nij = n-ij
                  npijt = np-ijt
                  if(ip.eq.2) then
c     ......................................................
c     .  calculate the scattered field for theta incident  .
c     .    polarization (perpendicular to x-y plane)       .
c     .  s(1), s(2), s(3) = radial, theta, phi components  .
c     ......................................................
      s(1) = s(1)+cs*cd1(npijt,im)*rc                                   eq 3.10a
      s(2) = s(2)+(p2*cd1(nij,im)*cb+cs*p1*cd1(npijt,im)*b)             eq 3.10b
      s(3) = s(3)-(p1*cd1(nij,im)*sb+sn*p2*cd1(npijt,im)*b)             eq 3.10c
                  else
c     ......................................................
c     .  calculate the scattered field for phi incident    .
c     .    polarization (parallel to x-y plane)            .
c     .  s(1), s(2), s(3) = radial, theta, phi components  .
c     ......................................................
      s(1) = s(1)-sn*cd2(npijt,im)*rc                                   eq 3.11a
      s(2) = s(2)-(p2*cd2(nij,im)*sb+sn*p1*cd2(npijt,im)*b)             eq 3.11b
      s(3) = s(3)-(p1*cd2(nij,im)*cb+cs*p2*cd2(npijt,im)*b)             eq 3.11c
                  end if
                end if
60            continue
70          continue
c     ......................................................
c     .  calculate the intensity (electric field squared)  .
c     ......................................................
            e2 = abs(s(1))**2+abs(s(2))**2+abs(s(3))**2                 eq 3.55
c     ...............................................
c     .  store intensity at (xg,yg) in array ac(*)  .
c     ...............................................
            if(iphi.eq.1) then
              ac(jg) = e2
            else
              ac(npm-jg) = e2
            end if
80        continue
c     .......................
c     .  increment x value  .
c     .......................
          xg = xg+dltx
90      continue
c     ......................................
c     .  write out data for all xg values  .
c     .    (for given yg value)            .
c     ......................................
        if(idim.eq.2) then
          xp = -1.0
          do 100 j = 1,npnts
            write(9,240) xp,ac(j)
            xp = xp+dltx
100       continue
        else
          write(9,250) (ac(j),j=1,npnts)
        end if
c     .......................
c     .  increment y value  .
c     .......................
        yg = yg+dlty
110   continue
      close(unit=9)
      stop
200   format(2i4,4f8.4)
210   format('...........................................',/,
     1       '.  calculate 2D or 3D internal intensity  .',/,
     2       '.  output is written to t8.dat            .',/,
     3       '...........................................',//,
     4       'polarization: parallel(1) or perpendicular (2)',/,
     5       '              to the grid ( x - y ) plane',/,
     6       'number of points: npnts (multiple of 4)',/,
     7       'graph dimension: idim (2 or 3)',/)
220   format(/,'2D dimension:',i4)
230   format(/,'3D grid dimension:',i4,' x',i4)
240   format(2e14.6)
250   format(e14.6)
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
      subroutine besj(z,bslcmp,nc)
c     ......................................................
c     .  calculate Bessel functions of the first kind      .
c     .  bslcmp = Bessel function of the first kind        .
c     .  z = complex argument                              . 
c     .  nc = number of orders (0 to nc-1)                 .
c     .  the order of the Bessel functions is incremented  .
c     .    by one in the bslcmp(*) array                   .
c     ......................................................
      complex bslcmp(nc),t(3),z,alpha
c     ...................................................
c     .  set the starting order for downward recursion  .
c     ...................................................
      x = abs(z)
      nst = nc+int((101.0+x)**.5)
c     ....................................................
c     .  the t(*) array is used to recur down to the     .
c     .    two highest order functions that are needed   .
c     .  set starting values for the two highest orders  .
c     .    nst and nst-1
c     ....................................................
      t(3) = 0.0
      t(2) = 1.0e-35
c     ...................................................
c     .  recur downward to obtain orders nc-1 and nc-2  .
c     ...................................................
      do 10 i = nst-1,nc-1,-1
        ri = real(i)
        t(1) = (2.0*ri+1.0)*t(2)/z-t(3)
        t(3) = t(2)
        t(2) = t(1)
10    continue
c     ...............................................
c     .  continue downward recursion to order zero  .
c     ...............................................
      bslcmp(nc) = t(3)
      bslcmp(nc-1) = t(2)
      do 20 i = nc-2,1,-1
        ri = real(i)
        bslcmp(i) = (2.0*ri+1.0)*bslcmp(i+1)/z-bslcmp(i+2)
20    continue
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
      alpha = sin(z)/(z*bslcmp(1))                                      eq 3.73
      do 30 k = 1,nc
        bslcmp(k) = bslcmp(k)*alpha
30    continue
      return
      end
      subroutine addprc 
c     .........................................................
c     .  calculate the internal field expansion coefficients  .
c     ......................................................... 
      complex a,ab1(50),ab2(50),ci,c1,c2,cd1,cd2
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,a(50,50)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x 
      common /uvccom/ cd1(50,26),cd2(50,26),p(26,26)
      dimension pnmllg(26)
      logical first
      data first /.true./
c     ...........................
c     .  on first entry set ci  .
c     ...........................
      if(first) then
        ci = (0.0,1.0)
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
c     ........................................
c     .  calculate the associated Legendre   .
c     .    functions for the incident angle  .
c     .    special case: theta = 90 degrees  .
c     ........................................
      theta = pi/2.0
      sinth = 1.0
      costh = 0.0
      call genlgp(pnmllg,nranki) 
c     ..........................................................
c     .  store the Legendre functions for all azimuthal modes  .
c     .    for later use in calculating the scattered field    .
c     .    in the x-y (theta = 90 degrees) plane               .
c     ..........................................................
      do 10 i = 1,nranki
        p(i,kmv+1) = pnmllg(i)
10    continue
c     ..................................................
c     .  calculate the incident field coefficients:    .
c     .    ab1(*) = theta polarization                 .
c     .    ab2(*) = phi polarization                   .
c     .  note: ab1(*) and ab2(*) used in scattered     .
c     .        field programs are j times ab1(*) and   .
c     .        ab2(*) used in internal field programs  .            
c     ..................................................
        do 20 n = 1,nrank 
          if(n.gt.ij) then
            np = n+nrank 
            cn = real(n)
            n1 = n+1 
            c1 = 4.0*ci**n 
            c2 = 4.0*ci**n1 
            p1 = -(cn+cmv)*pnmllg(n) 
            p2 = cmv*pnmllg(n1) 
            ab1(n-ij) = -c2*p2
            ab1(np-ijt) = -c1*p1
            ab2(n-ij) = c2*p1
            ab2(np-ijt) = c1*p2
          end if
20      continue 
c     ....................................................
c     .                                       -1         .
c     .  the internal field coefficients = [A]   times   .
c     .    the incident field coefficients               .
c     .  cd2(ns+1) to cd2(ns2) is -d(omn') per (3.8b)    .
c     .  store the coefficients for all azimuthal modes  .
c     ....................................................
      call minv(ab1,ab2,ns2)                                            eq 3.8
      do 30 i = 1,ns2
        cd1(i,kmv+1) = ab1(i)
        cd2(i,kmv+1) = ab2(i)
30    continue 
      return
      end
      subroutine minv(ab1,ab2,n)
c     ...............................................
c     .             _ _       _         _ -1  _ _   .
c     .            |   |     |           |   |   |  .
c     .            | c |     |           |   | a |  .
c     .  calculate | - |  =  |     A     |   | - |  .
c     .            | d |     |           |   | b |  .
c     .            |_ _|     |_         _|   |_ _|  .
c     .                                             .
c     .  using Gauss-Jordan reduction               .
c     ...............................................
      common /mtxcom/ nrank,nranki,a(50,50)
      complex a,ab1(50),ab2(50),aijmax,arat
      dimension ls(50)
c     ............................
c     .  start reduction of [a]  .
c     ............................
      do 50 i = 1,n
c     ..........................................................
c     .  search for the maximum element in the ith row of [a]  . 
c     ..........................................................
        aijmax = a(i,1)
        jmax = 1
        do 10 j = 2,n
          if(abs(a(i,j)).gt.abs(aijmax)) then
            aijmax = a(i,j)
            jmax = j
          end if
10      continue
c     ........................................................
c     .  normalize the ith row of [a] and the rhs by aijmax  .
c     ........................................................
        do 20 j=1,n
          a(i,j) = a(i,j)/aijmax
20      continue
        ab1(i) = ab1(i)/aijmax
        ab2(i) = ab2(i)/aijmax
        do 40 k = 1,n
c     .......................................................
c     .  use row transformations to obtain zeros above and  .
c     .    below the jmax element of the ith row of [a] -   .
c     .    apply the same row transformations to the rhs    .
c     .......................................................
          if(k.ne.i) then
            arat = -a(k,jmax)
            do 30 j = 1,n
              if(abs(a(i,j)).gt.0.0) then
                a(k,j) = arat*a(i,j)+a(k,j)
              end if
30          continue
            a(k,jmax) = 0.0
            ab1(k) = arat*ab1(i)+ab1(k)
            ab2(k) = arat*ab2(i)+ab2(k)
          end if
40      continue
c     ........................................................
c     .  store row counter (i) in array ls(*), such that     .
c     .    ls(*) contains the location of the pivot (unity)  .
c     .    element of each column (after reduction)          .
c     ........................................................
        ls(jmax) = i
50    continue
c     ....................................................
c     .  the reduction of [a] is complete - perform      .
c     .    row interchanges as indicated in array ls(*)  .
c     ....................................................
      do 70 i = 1,n
        k = i
c     .........................................................
c     .  put the integer value in ls(k) into k                .
c     .                                                       .
c     .  if k is less than i, then that row has already been  .
c     .    involved in an interchange so iterate k = ls(k)    .
c     .    until a value of k greater than i (corresponding   . 
c     .    to a row stored above the ith row) is obtained     .
c     .........................................................
60      k = ls(k)
        if(k.lt.i) go to 60
        if(k.gt.0) then
          arat = ab1(i)
          ab1(i) = ab1(k)
          ab1(k) = arat
          arat = ab2(i)
          ab2(i) = ab2(k)
          ab2(k) = arat
        end if
70    continue
      return
      end
