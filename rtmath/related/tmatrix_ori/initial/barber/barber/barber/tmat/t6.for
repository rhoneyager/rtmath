      program T6
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ..................................................................
c     .  calculate the elements of the scattering matrix for an        .
c     .    ensemble of identical randomly oriented axisymmetric        .
c     .    particles following the definition of S in Chapter 3.3      .
c     .    of C.E. Bohren and D.R. Huffman, Absorption and Scattering  .
c     .    of Light by Small Particles (Wiley-Interscience,New York,   .
c     .    1983) - the scattering matrix is normalized to P as in      .
c     .    S. Asano and M. Sato, "Light Scattering by Randomly         .
c     .    Oriented Spheroidal Particles,"Appl. Opt.,19,962-974(1980)  .
c     .  also calculate the scattering, absorption and extinction      .
c     .    efficiencies                                                .
c     .  inputs:  ng = number of integration points (theta points)     .
c     .           dlt = increment in scattering angle (2 degree min)   .
c     .                                                               .
c     .  program is currently configured for mirror symmetric         .
c     .    particles.  program modification for particles without     .
c     .    mirror symmetry is achieved by adding and deleting the     .
c     .    statements noted in columns 73-80                          .
c     ..................................................................
      complex tmat,acans,s1,s2,s3,s4,ci,tstore(24600)
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25) 
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,ic
      common /outcom/ acans(181,2,2),sc,ex,nuang
      dimension phi(17),ia(2),s(91,6),sp(91,6),ss(91,6),cstore(350)
      dimension wt(25),asc(25)
      data phi/0.0,90.0,45.0,22.5,67.5,11.25,33.75,56.25,78.75,5.625, 
     116.875,28.125,39.375,50.625,61.875,73.125,84.375/ 
c     ...........................
c     .  set program constants  .
c     ...........................
      dtr = .017453292519943 
      rtd = 57.2957795131 
      pi = 3.14159265358979 
      ci = (0.0,1.0)
c     ...................................................
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
      read(12,200) nm,nrank,x
      close(unit=12)
      nranki = nrank+1
      write(6,210)
      write(6,*) 'enter number of integration sections, dlt'
      read(5,*) ng,dlt
      nuang = int(360.0/dlt)+1
      nang = (nuang-1)/2+1 
c     .......................................................
c     .  read the T-matrix and normalization constants for  .
c     .    all m and store in linear arrays for later use   .
c     .......................................................
      kstrt = 0
      kstrc = 0
      do 8 im = 1,nm
        kmv = im-1
        ij = kmv-1
        if(ij.lt.0) ij = 0
        ns = nrank-ij
        ns2 = 2*ns
        read(11) tmat,cmxnrm
        do 4 j = 1,ns2
          do 2 i = 1,ns2
            kstrt = kstrt+1
            tstore(kstrt) = tmat(i,j)
2         continue
4       continue
        do 6 k = 1,ns
          kstrc = kstrc+1
          cstore(kstrc) = cmxnrm(k)
6       continue
8     continue
        close(unit=11)
c     .....................................................
c     .  set the integration points and weighting values  .
c     .....................................................
      clim = 1.0                                                        delete
      call gauss(wt,asc,ng,0.0,1.0)                                     delete
c      clim = 2.0                                                        add
c      call gauss(wt,asc,ng,-1.0,1.0)                                    add
c     .......................................................
c     .  clear the result accumulators for the integration  .
c     .    over theta orientations (final results)          .
c     .......................................................
      do 20 iz = 1,nang 
        do 10 ir = 1,6
          ss(iz,ir) = 0.0
10      continue           
20    continue 
      scat = 0.0
      ext = 0.0
c     .......................................................
c     .  enter a loop to integrate over theta orientations  .
c     .......................................................
      do 150 ith = 1,ng 
c     ......................................................
c     .  set theta corresponding to the integration point  .
c     ......................................................
        angint = acos(asc(ith))
c     ...........................................
c     .  clear the result accumulators for the  .
c     .    integration over phi orientations    .
c     ...........................................
        do 40 iz = 1,nang 
          do 30 ir = 1,6
            s(iz,ir) = 0.0
30        continue     
40      continue 
        sc1 = 0.0
        ex1 = 0.0
c     .......................................................
c     .  enter a loop to integrate over phi orientations    .
c     .    set values in the first quadrant - use symmetry  .
c     .    to obtain results for other quadrants            .
c     .  up to 64 equivalent phi orientations will be used  .
c     .  set switches for convergence testing               .
c     .......................................................
        km = 1 
        kt = 0 
        ktst = 4 
        do 110 iphi = 1,17 
c     .............
c     .  set phi  .
c     .............
          anginp = phi(iphi)*dtr 
c     ................................................................
c     .  enter a loop for each azimuthal mode m and return acans     .
c     .  acans(*,2,2), acans(*,1,1), acans(*,1,2), acans(*,2,1)      .
c     .  second subscript = scattered polarization | 1 = theta pol.  .
c     .  third subscript = incident polarization   | 2 = phi pol.    .
c     ................................................................
          kstrt = 0
          kstrc = 0
          do 60 im = 1,nm 
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
            kmv = im-1
            cmv = real(kmv)
            prodm = 1.0 
            if(kmv.gt.0) then
              quanm = cmv 
              do 50 i = 1,kmv 
                quanm = quanm+1.0 
                prodm = quanm*prodm/2.0 
50            continue 
            end if
            twm = 2.0*cmv 
            ij = kmv-1
            if(ij.lt.0) ij = 0
            ns = nrank-ij
            ns2 = 2*ns
c     .......................................................
c     .  retrieve the T-matrix and normalization constants  .
c     .    from the linear arrays for each m                .
c     .......................................................
            do 56 j = 1,ns2
              do 54 i = 1,ns2
                kstrt = kstrt+1
                tmat(i,j) = tstore(kstrt)
54            continue
56          continue
            do 58 k = 1,ns
              kstrc = kstrc+1
              cmxnrm(k) = cstore(kstrc)
58          continue
c     ............................................................
c     .  calculate the scattered field over 360 degrees and      .
c     .    the efficiencies for each angint, anginp orientation  .
c     ............................................................
            call addprc 
60        continue 
c     ..............................................
c     .  add angular scattering contributions      .
c     .    for phi-symmetric orientations          .
c     .  anginp = 0 and 180 degrees give one       .
c     .    regular and one reversed contribution   .
c     .  other anginp orientations give two        .
c     .    regular and two reversed contributions  .
c     ..............................................
          if(iphi.ge.3) km = 2 
          cm = real(km) 
          kr = nuang 
          do 80 j = 1,nang 
            ia(1) = j
            ia(2) = kr
            do 70 ja = 1,2
c     ......................................................
c     .  convert the T-matrix notation to the elements of  .
c     .    the amplitude scattering matrix S1, S2, S3, S4  .
c     .  acans(*,2,2) = k*F1     acans(*,1,1) = k*F2       .
c     .  acans(*,1,2) = k*F3     acans(*,2,1) = k*F4       .
c     ......................................................
              s1 = -ci*acans(ia(ja),2,2)                                eq 3.50a
              s2 = -ci*acans(ia(ja),1,1)                                eq 3.50b
              s3 = ci*acans(ia(ja),1,2)                                 eq 3.50c
              s4 = ci*acans(ia(ja),2,1)                                 eq 3.50d
c     .....................................................
c     .  calculate six elements of the scattering matrix  .
c     .    s(*,1) = S11, s(*,2) = S12, s(*,3) = S22       .
c     .    s(*,4) = S33, s(*,5) = S34, s(*,6) = S44       .
c     .....................................................
              t1 = cabs(s1)**2
              t2 = cabs(s2)**2
              t3 = cabs(s3)**2
              t4 = cabs(s4)**2
              s(j,1) = s(j,1)+.5*cm*(t1+t2+t3+t4)
              s(j,2) = s(j,2)+.5*cm*(t2-t1+t4-t3)
              s(j,3) = s(j,3)+.5*cm*(t2+t1-t4-t3)
              t1 = real(s1*conjg(s2))
              t2 = real(s3*conjg(s4))
              s(j,4) = s(j,4)+cm*(t1+t2)
              s(j,6) = s(j,6)+cm*(t1-t2)
              t1 = aimag(s2*conjg(s1))
              t2 = aimag(s4*conjg(s3))
              s(j,5) = s(j,5)+cm*(t1+t2)
70          continue
            kr = kr-1 
80        continue 
c     .......................................................
c     .  accumulate the efficiencies over phi orientations  .
c     .......................................................
          sc1 = sc1+2.0*cm*sc
          ex1 = ex1+2.0*cm*ex
c     ............................................
c     .  calculate the equivalent number of phi  .
c     .    orientations, kt = 2,4,8,12,16, ...   .
c     ............................................
          kt = kt+2*km 
c     ..............................................................
c     .  check to see if convergence over phi orientations should  .
c     .    be tested - testing is done for kt = 4,8,16,32,64       .
c     ..............................................................  
          if(kt.eq.ktst) then
            ct = real(kt) 
            nc = 0 
            do 100 k = 1,nang 
c     ..............................................
c     .  calculate the average angular scattering  .
c     .    over phi orientations for S11           .
c     ..............................................
              st = s(k,1)/ct
c     ......................................................
c     .  check convergence over phi orientations at each   .
c     .    scattering angle for S11                        .
c     .  skip the convergence test the first time through  .
c     ......................................................
              if(ktst.gt.4) then
                if(abs(st-sp(k,1)).le.(.01*st)) nc = nc+1
              end if
c     .......................................................
c     .  calculate the average angular scattering over phi  .
c     .    orientations for all scattering matrix elements  .
c     .......................................................
              do 90 ir = 1,6
                sp(k,ir) = s(k,ir)/ct                                   eq 3.45
90            continue
100         continue 
            ktst = 2*ktst 
c     .................................................
c     .  if the angular scattering is converged over  .
c     .    phi orientations then exit the phi loop    .
c     .................................................
            if(nc.eq.nang) go to 120
          end if
110     continue 
c     .................................................
c     .  calculate the percentage of the scattering   .
c     .    angles at which the results are converged  .
c     .    to one per cent over phi orientations      .
c     .................................................
120     cc = 100.0*real(nc)/real(nang)
        angtpr = angint*rtd
        write(6,220) angtpr,kt,cc
c     .......................................................
c     .  accumulate the angular scattering over theta       .
c     .    orientations for all scattering matrix elements  .
c     .......................................................
        do 140 is = 1,nang
          do 130 ir = 1,6
            ss(is,ir) = ss(is,ir)+wt(ith)*sp(is,ir)/clim                eq 3.46
130       continue
140     continue 
c     .........................................................
c     .  accumulate the efficiencies over theta orientations  .
c     .........................................................
        scat = scat+wt(ith)*sc1/ct/clim
        ext = ext+wt(ith)*ex1/ct/clim
150   continue 
c     .................................................
c     .  convert the scattering matrix elements S to  .
c     .    normalized scattering matrix elements P    .
c     .................................................
      do 170 k = 1,nang
        do 160 ir = 1,6
          ss(k,ir) = 4.0*ss(k,ir)/(scat*x**2)                           eq 3.52
160     continue
170   continue
      ab = ext-scat                                                     eq 3.22
      if(ab.lt.1.0e-08) ab = 0.0
      write(6,230) scat,ab,ext
      open(unit=9,file='t6.dat')
      rewind 9
      do 180 i = 1,nang
        ri = real(i-1)
        ang = ri*dlt
        pl = -ss(i,2)/ss(i,1)
        px = 1.0-ss(i,3)/ss(i,1)
        p33 = ss(i,4)/ss(i,1)
        p34 = ss(i,5)/ss(i,1)
        p44 = ss(i,6)/ss(i,1)
        write(9,240) ang,ss(i,1),pl,px,p33,p34,p44
180   continue
      close(unit=9)
200   format(2i4,f8.4)
210   format('..........................................',/,
     1       '.  calculate scattering matrix elements  .',/,
     2       '.  P11, -P12/P11, 1-P22/P11, P33/P11,    .',/,
     3       '.  P34/P11, and P44/P11                  .',/,
     4       '.  output is written to t6.dat           .',/,
     5       '..........................................',//,
     6       'dlt: increment in scattering angle (2 degree min)',/)
220   format('theta =',f6.2,'  kt =',i3,'  convergence =',f6.1,' %')
230   format(/,' scat xsect = ',1pe13.4,/'  abs xsect = ',e13.4,/
     1'  ext xsect = ',e13.4)
240   format(f6.2,6e12.4) 
      stop 
      end 
      subroutine gauss(wt,asc,n,aa,bb) 
c     ..................................................................
c     .  calculate abscissas and weights for n-point Gaussian          .
c     .    quadrature over integration limits of aa to bb              .
c     .  aa  = lower integration limit                                 .
c     .  bb  = upper integration limit                                 .
c     .  n   = number of integration points                            .
c     .  asc = abscissas                                               .
c     .  wt  = weights                                                 .
c     .                                                                .
c     .  a well-documented program which performs the same function    .
c     .    is given in Chapter 4.5 of W.H. Press, B.P. Flannery,       . 
c     .    S.A. Teukolsky and W.T. Vetterling, Numerical Recipes -     .
c     .    The Art of Scientific Computing (Cambridge University       .
c     .    Press, Cambridge, 1986)                                     .
c     .                                                                .
c     .  the results (either program) compare with Table 25.4 in M.    .
c     .    Abramowitz and I.A. Stegun, Handbook of Mathematical Func-  .
c     .    tions (National Bureau of Standards, Washington, 1964)      .
c     ..................................................................
      dimension wt(n),asc(n) 
      data pi,const,tol/3.14159265358979,.148678816357,1.0e-07/ 
c     ........................................................
c     .  the tolerance (tol) may be decreased to 1.0d-10 or  .
c     .    less when double precision variables are used     .
c     ........................................................
      data c1,c2,c3,c4/.125,-.0807291666,.2460286458,-1.824438767/ 
      if(n.eq.1) then
        asc(1) = 0.5773502692 
        wt(1) = 1.0 
        return 
      end if
      cn = real(n) 
      ndiv2 = n/2 
      np1 = n+1 
      cnn1 = cn*(cn+1.0)
      appfct = 1.0/sqrt((cn+.5)**2+const) 
      con1 = .5*(bb-aa) 
      con2 = .5*(bb+aa) 
      do 30 k = 1,ndiv2 
        b = (real(k)-.25)*pi 
        bisq = 1.0/b**2 
        bfroot = b*(1.0+bisq*(c1+bisq*(c2+bisq*(c3+c4*bisq)))) 
        xi = cos(appfct*bfroot) 
10        x = xi
          pm2 = 1.0 
          pm1 = x 
          do 20 in = 2,n 
            rn = real(in)  
            p = ((2.0*rn-1.0)*x*pm1-(rn-1.0)*pm2)/rn 
            pm2 = pm1 
            pm1 = p 
20        continue
          pm1 = pm2 
          aux = 1.0/(1.0-x**2) 
          der1p = cn*(pm1-x*p)*aux 
          der2p = (2.0*x*der1p-cnn1*p)*aux 
          ratio = p/der1p 
          xi = x-ratio*(1.0+ratio*der2p/(2.0*der1p)) 
        if(abs(xi-x).gt.tol) go to 10
        asc(k) = -xi 
        wt(k) = 2.0*(1.0-xi**2)/(cn*pm1)**2 
        asc(np1-k) = -asc(k) 
        wt(np1-k) = wt(k) 
30    continue
      if(mod(n,2).ne.0) then
        asc(ndiv2+1) = 0.0 
        nm1 = n-1 
        nm2 = n-2 
        prod = cn 
        do 40 k = 1,nm2,2 
          prod = prod*real(nm1-k)/real(n-k)
40      continue      
        wt(ndiv2+1) = 2.0/prod**2 
      end if
      do 50 k = 1,n 
        asc(k) = con1*asc(k)+con2 
        wt(k) = con1*wt(k) 
50    continue
      return 
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
        plc = (cnmul*costh*plb-cnmm*pla)/cnm                            eq 3.78
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
c     ............................................................
c     .  calculate the polarized scattering and the scattering,  .
c     .    absorption, and extinction efficiencies               .
c     .  calculations are performed in the particle frame        .
c     ............................................................
      complex tmat,ab1(50),ab2(50),fg1(50),fg2(50),ci,c1,c2,cim,acans,
     1fg1n,fg1np,fg2n,fg2np
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),cmxnrm(25)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,ic
      common /outcom/ acans(181,2,2),sc,ex,nang
      dimension tang(181),pang(181),cosb(181),sinb(181)
      dimension pnmllg(26),clrtot(1448),uu1(2),uu2(2)
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
      if(kmv.eq.0) then
        s = 0.0
        do 10 i = 1,1448
          clrtot(i) = 0.0
10      continue
        call genang(nang,tang,pang,cosb,sinb)
c     .......................................................
c     .  transform the incident field polarization vector   .
c     .    from the laboratory frame to the particle frame  .
c     .    uu1 = theta component and uu2 = phi component    .
c     .  uu1(1), uu2(1) = theta polarization incident       .
c     .  uu1(2), uu2(2) = phi polarization incident         .
c     .                                                     .
c     .  the conversion to the particle frame is delayed    .
c     .    for incorporation into the scattered field       .
c     .    coefficients fg1 and fg2 - earlier calculations  .
c     .    are then common to both incident polarizations   .
c     .......................................................
        uu1(1) = cosb(1)
        uu2(1) = sinb(1)
        uu1(2) = -sinb(1)
        uu2(2) = cosb(1)
      end if
c     .........................................................
c     .  calculate the associated Legendre functions for the  .
c     .    incident angle = tang(1) in the body frame         .
c     .........................................................
c     ...................................................
c     .  set indices for matrix compression when n < m  .
c     .    note: useful only when m > 1                 .
c     ...................................................
      ij = kmv-1
      if(ij.lt.0) ij = 0
      ijt = 2*ij
      ns = nrank-ij
      ns2 = 2*ns
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
        ab1(n-ij) = c1*p2 
        ab1(np-ijt) = -c2*p1 
        ab2(n-ij) = -c1*p1 
        ab2(np-ijt) = c2*p2 
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
c     ...............................................................
      do 70 iu = 1,nang 
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
        do 60 n = 1,nrank 
          if(n.le.ij) go to 60
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
c     ...................................................................
c     .  calculate the theta and phi polarized scattered fields in the  .
c     .    particle frame for theta and phi polarized incident fields   .
c     .  acans = kF, where F is a component of the vector               .
c     .    far-field amplitude                                          .
c     .  acans(*,2,2), acans(*,1,1), acans(*,1,2), acans(*,2,1)         .
c     .  second subscript = scattered polarization | 1 = theta pol.     .
c     .  third subscript = incident polarization   | 2 = phi pol.       .
c     ...................................................................
          do 50 ip = 1,2
            fg1n = fg1(nij)*uu1(ip)
            fg1np = fg1(npijt)*uu1(ip)
            fg2n = fg2(nij)*uu2(ip)
c     .......................................
c     .  fg2(npijt) is -g(omn) per (3.12b)  .
c     .......................................
            fg2np = fg2(npijt)*uu2(ip)
c     .............................................
c     .  solve for the theta polarized scattered  .
c     .    field in the particle frame            .
c     .............................................
            acans(iu,1,ip) = acans(iu,1,ip)+cim*(fg1n*dd+ci*fg1np*bb    eq 3.17a
     1                       -fg2n*cc-ci*fg2np*aa)*cmxnrm(nij)          eq 3.18a
c     ...........................................
c     .  solve for the phi polarized scattered  .
c     .    field in the particle frame          .
c     ...........................................
            acans(iu,2,ip) = acans(iu,2,ip)-cim*(fg1n*aa+ci*fg1np*cc    eq 3.17b
     1                       +fg2n*bb+ci*fg2np*dd)*cmxnrm(nij)          eq 3.18b
50        continue
60      continue 
70    continue 
c     ....................................................
c     .  calculate the scattering cross section and      .
c     .    accumulate for each azimuthal mode m          .
c     .  calculate for theta incident polarization only  .
c     ....................................................
      do 80 n = 1,ns
        np = n+ns
        s = s+(abs(fg1(n)*uu1(1))**2+abs(fg1(np)*uu1(1))**2
     1        +abs(fg2(n)*uu2(1))**2+abs(fg2(np)*uu2(1))**2)*cmxnrm(n)  eq 3.21
80    continue
c     .........................................
c     .  transform to the laboratory frame    .
c     .  and normalize the converged results  .
c     .........................................  
      if(kmv.eq.(nm-1)) then
        do 100 j = 1,nang 
          do 90 ip = 1,2
            c1 = acans(j,1,ip)
            c2 = acans(j,2,ip)
            acans(j,1,ip) = cosb(j)*c1+sinb(j)*c2                       eq 3.37a
            acans(j,2,ip) = cosb(j)*c2-sinb(j)*c1                       eq 3.37b
90        continue
100     continue 
c     ............................................
c     .  calculate the extinction cross section  .
c     .    for theta incident polarization only  .
c     ............................................
        ex = aimag(acans(1,1,1))*4.0/x**2                               eq 3.20
        sc = s/x**2
      end if
      return
      end
