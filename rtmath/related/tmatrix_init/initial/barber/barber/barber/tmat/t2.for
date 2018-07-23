      program T2
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .................................................................
c     .  calculate the differential scattering cross section and      .
c     .    the scattering, absorption and extinction efficiencies     .
c     .  inputs:  anglab = angle of the E-field polarization          .
c     .                      vector in the x-y plane, degrees         .
c     .         | dlt = increment in scattering angle (2 degree min)  .
c     .  degrees| angint = theta orientation in the laboratory frame  .
c     .         | anginp = phi orientation in the laboratory frame    .
c     .................................................................
      complex tmat
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,ic 
      common /outcom/ v(361),h(361),sc,ex,nang
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
c     .        x     =  size parameter (ka)              .
c     ....................................................
      open(unit=11,file='t',form='unformatted')
      rewind 11
      open(unit=12,file='case')
      rewind 12
      read(12,100) nm,nrank,x
      close(unit=12)
      nranki = nrank+1
      write(6,110)
      write(6,*) 'enter anglab, delta angle'
      read(5,*) anglab,dlt
      anglab = anglab*dtr
c     .................................................................
c     .  utheta and uphi are the amplitudes of the incident field in  .
c     .    the theta (parallel) and phi (perpendicular) directions    .
c     .................................................................
      utheta = cos(anglab)
      uphi = sin(anglab)
      nang = int(360.0/dlt)+1
      write(6,120)
      write(6,*) 'enter angint,anginp'
      read(5,*) angint,anginp
      anginp = anginp*dtr
      if(angint.eq.0.0.or.angint.eq.180.0) then
c     ..............................................................
c     .  calculation for angint = 0 or 180 degrees ( m = 1 only )  .
c     ..............................................................
        ic = 1
        angint = angint*dtr
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
        kmv = 1 
        cmv = 1.0
        prodm = 1.0 
        twm = 2.0 
c     ...............................................................
c     .  read the stored arrays twice ( m = 0 and m = 1) to obtain  .
c     .    the T-matrix and normalization constants for m = 1       .
c     ...............................................................
        read(11) tmat,cmxnrm
        read(11) tmat,cmxnrm
c     .......................................................
c     .  calculate the angular scattering and efficiencies  .
c     .......................................................
        call addprc 
      else
c     ...........................................................
c     .  calculation for angint not 0 or 180 degrees ( all m )  .
c     ...........................................................
        ic = 2
        angint = angint*dtr
c     ............................................
c     .  enter a loop for each azimuthal mode m  .
c     ............................................
        do 20 im = 1,nm 
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
10          continue 
          end if
        twm = 2.0*cmv 
c     ..............................................................
c     .  read the T-matrix and normalization constants for each m  .
c     ..............................................................
        read(11) tmat,cmxnrm
c     .......................................................
c     .  calculate the angular scattering and efficiencies  .
c     .......................................................
        call addprc 
20    continue 
      end if
      open(unit=9,file='t2.dat')
      rewind 9
      do 30 i = 1,nang
        ri = real(i-1)
        ang = ri*dlt
        write(9,130) ang,h(i),v(i)
30    continue
      close(unit=9)
      close(unit=11)
      ab = ex-sc                                                        eq 3.22
      if(ab.lt.1.0e-8) ab = 0.0
      write(6,140) sc,ab,ex
      stop
100   format(2i4,f8.4)
110   format('...................................................',/,
     1       '.  calculate angular scattering and efficiencies  .',/,
     2       '.  output (par.  perp.) is written to t2.dat      .',/,
     3       '...................................................',//,
     4       'anglab: angle of the E-field polarization',/,
     1       '        vector in the x-y plane, degrees',/, 
     2       'dlt: increment in scattering angle (1 degree min)',/)
120   format(/,'angint: theta orientation in the lab frame, degrees',/,
     1         'anginp: phi orientation in the lab frame, degrees',/)
130   format(3e14.6)
140   format('scat xsect = ',1pe13.4,/,' abs xsect = ',e13.4,/,
     1' ext xsect = ',e13.4)
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
      common /uvccom/ angint,anginp,utheta,uphi,ic
      dimension tang(n),pang(n),cosb(n),sinb(n)
c     ..............................................................
c     .  dltang calculation assumes scattering over 360.0 degrees  .
c     ..............................................................
      dltang = 2.0*pi/real(n-1)
      nswtch = (n-1)/2+1
      sint = sin(angint)
      cost = cos(angint)
      sinp = sin(anginp)
      cosp = cos(anginp)
c     ...............................................................
c     .  calculate scattering angles in the particle frame corres-  .
c     .    ponding to scattering angles in the laboratory frame     .
c     ...............................................................
      do 10 i = 1,n 
        ts = dltang*real(i-1)
        if(i.le.nswtch) then
          ps = 0.0
        else  
          ts = 2.0*pi-ts
          ps = pi
        end if  
        sints = sin(ts)
        costs = cos(ts)
        cosps = cos(ps)
        x = -cosp*cost*sints*cosps+sint*costs                           eq 3.29a
        y = sinp*sints*cosps                                            eq 3.29b
        z = cosp*sint*sints*cosps+cost*costs                            eq 3.29c
c     .................................................................
c     .  calculate the theta scattering angles in the particle frame  .
c     .................................................................
        r = sqrt(x**2+y**2)
        tang(i) = 0.0
        if(z.ne.0.0.or.r.ne.0.0) tang(i) = atan2(r,z)                   eq 3.30a
c     ...............................................................
c     .  calculate the phi scattering angles in the particle frame  .
c     ...............................................................
        pang(i) = 0.0
        if(x.ne.0.0.or.y.ne.0.0) pang(i) = atan2(y,x)                   eq 3.30b
        if(pang(i).lt.0.0) pang(i) = 2.0*pi+pang(i)
c     .................................................................
c     .  calculate the elements of the transform matrix for the       .
c     .    polarization vectors                                       . 
c     .                                                               .
c     .  sint  = sine   (particle theta orientation angle)            .
c     .  cost  = cosine (particle theta orientation angle)            .
c     .  sinp  = sine   (particle phi orientation angle)              .
c     .  cosp  = cosine (particle phi orientation angle)              .
c     .                                                               .
c     .  sints = sine   (theta scattering angle in laboratory frame)  .
c     .  costs = cosine (theta scattering angle in laboratory frame)  .
c     .  cosps = cosine (phi scattering angle in laboratory frame)    .
c     .                                                               .
c     .  pang(i) = phi scattering angle in particle frame             .
c     .................................................................
        cosb(i) = cosps*sinp*cost*sin(pang(i))
     1            -cosps*cosp*cos(pang(i))                              eq 3.39a
        sinb(i) = costs*cosps*cosp*cost*sin(pang(i))
     1            +sints*sint*sin(pang(i))+costs*cosps*sinp*cos(pang(i))eq 3.39b
10    continue 
      return 
      end 
      subroutine addprc 
c     ...............................................................
c     .  calculate the differential scattering cross section and    .
c     .    the scattering, absorption, and extinction efficiencies  .
c     .  calculations are performed in the particle frame           .
c     ...............................................................
      complex tmat,ab1(50),ab2(50),fg1(50),fg2(50),ci,c1,c2,cim,
     1acans(361,2)
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,ic 
      common /outcom/ v(361),h(361),sc,ex,nang
      dimension tang(361),pang(361),cosb(361),sinb(361)
      dimension pnmllg(26),clrtot(1444)
      equivalence (acans,clrtot)
      logical first
      data first /.true./
c     ............................................................
c     .  set constants the first time the subroutine is entered  .
c     ............................................................
      if(first) then
        ci = (0.0,1.0) 
        nr2 = 2*nrank
        snorm = 1.0/(pi*x**2)
        first = .false.
      end if
c     ................................................................
c     .  clear the result accumulators and calculate the coordinate  .
c     .    transformation variables for each new orientation         .
c     ................................................................  
      if(ic.eq.1.or.kmv.eq.0) then
        s = 0.0
        do 10 i = 1,1444
          clrtot(i) = 0.0
10      continue
        call genang(nang,tang,pang,cosb,sinb)
c     .......................................................
c     .  transform the incident field polarization vector   .
c     .    from the laboratory frame to the particle frame  .
c     .    uu1 = theta component and uu2 = phi component    .
c     .......................................................
        uu1 = cosb(1)*utheta-sinb(1)*uphi                               eq 3.38a
        uu2 = sinb(1)*utheta+cosb(1)*uphi                               eq 3.38b
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
c     .    incident angle = tang(1) in the particle frame     .
c     .........................................................
      theta = tang(1) 
      costh = cos(theta)
      sinth = sin(theta)
      if(tang(1).eq.pi) then
        theta = 0.0
        costh = -1.0
      end if
      call genlgp(pnmllg,nranki)
c     .................................................
c     .  calculate the incident field coefficients:   .
c     .    ab1(*) = theta polarization                .
c     .    ab2(*) = phi polarization                  .
c     .  note: ab1(*) and ab2(*) used in scattered     .
c     .        field programs are j times ab1(*) and   .
c     .        ab2(*) used in internal field programs  .            
c     .................................................
      do 20 n = 1,nrank 
        if(n.le.ij) go to 20
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
20    continue 
c     .............................................
c     .  the scattered field coefficients = -[T]  .
c     .    times the incident field coefficients  .
c     .  fg2(ns+1) to fg2(ns2) is -g(omn)         .
c     .............................................
      do 40 i = 1,ns2
        fg1(i) = 0.0 
        fg2(i) = 0.0 
        do 30 j = 1,ns2
          fg1(i) = fg1(i)-tmat(i,j)*ab1(j)                              eq 3.12a
          fg2(i) = fg2(i)-tmat(i,j)*ab2(j)                              eq 3.12b
30      continue 
40    continue 
c     ...............................................................
c     .  evaluate the scattered field at each scattering angle      .
c     .    theta = tang(*) and phi = pang(*) in the particle frame  .
c     .  results are accumulated in acans(*,*) for each             .
c     .    azimuthal mode m                                         .
c     .  acans = kF, where F is a component of the vector           . 
c     .    far-field amplitude                                      .
c     ...............................................................
      do 60 iu = 1,nang 
c     .................................................
c     .  calculate the associated Legendre functions  .
c     .    at each scattering angle                   .
c     .................................................
        theta = tang(iu)
        sinth = sin(theta)
        costh = cos(theta)
        if(tang(iu).eq.pi) then
          theta = 0.0
          costh = -1.0
        end if
        call genlgp(pnmllg,nranki) 
        phi = cmv*pang(iu)
        cosphi = cos(phi)
        sinphi = sin(phi)
        do 50 n = 1,nrank 
          if(n.le.ij) go to 50
          cn = real(n)
          np = n+nrank 
          n1 = n+1 
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
          acans(iu,1) = acans(iu,1)+cim*(fg1(nij)*dd+ci*fg1(npijt)*bb   eq 3.17a
     1                  -fg2(nij)*cc-ci*fg2(npijt)*aa)*cmxnrm(nij)      eq 3.18a
c     ...........................................
c     .  solve for the phi polarized scattered  .
c     .    field in the particle frame          .
c     .  fg2(npijt) is -g(omn) per (3.12b)      .
c     ...........................................
          acans(iu,2) = acans(iu,2)-cim*(fg1(nij)*aa+ci*fg1(npijt)*cc   eq 3.17b
     1                  +fg2(nij)*bb+ci*fg2(npijt)*dd)*cmxnrm(nij)      eq 3.18b
50      continue 
60    continue 
c     ................................................
c     .  calculate the scattering cross section and  .
c     .    accumulate for each azimuthal mode m      .
c     ................................................
      do 70 n = 1,ns
        np = n+ns
        s = s+(abs(fg1(n))**2+abs(fg1(np))**2
     1        +abs(fg2(n))**2+abs(fg2(np))**2)*cmxnrm(n)                eq 3.21
70    continue
c     .....................................
c     .  normalize the converged results  .
c     .....................................  
      if(ic.eq.1.or.kmv.eq.(nm-1)) then
        do 80 j = 1,nang 
          h(j) = snorm*abs(cosb(j)*acans(j,1)+sinb(j)*acans(j,2))**2    eq 3.37a
          v(j) = snorm*abs(cosb(j)*acans(j,2)-sinb(j)*acans(j,1))**2    eq 3.37b
80      continue 
        ex =(aimag(acans(1,1))*uu1+aimag(acans(1,2))*uu2)*4.0/x**2      eq 3.20
c     ............................................................
c     .  when angint = 180.0 degrees, the calculated extinction  .
c     .    efficiency will be negative so change the sign        .
c     ............................................................
        if(angint.eq.pi) ex = -ex
        sc = s/x**2
      end if
      return
      end
