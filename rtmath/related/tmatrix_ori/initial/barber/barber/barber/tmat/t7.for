      program T7
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .................................................................
c     .  calculate the normalized scattering cross section vs size    .
c     .    parameter for vertical (phi) and horizontal (theta)        .
c     .    polarizations for a particle in fixed or random orient-    .
c     .    ation                                                      .
c     .  inputs:  nrank = matrix order                                .
c     .           ntheta = integration steps                          .
c     .           cnk = starting size parameter                       .
c     .           cnke = ending size parameter                        .
c     .           npnts = number of size parameter values             .
c     .           ic = orientation case - fixed (1) or random (2)     .
c     .           aovrb = a/b ratio                                   .
c     .           cm = complex index of refraction, (real,imag)       .
c     .                (imag is positive for absorption)              .
c     .           anginc = angle of incidence, 0 to 180 degrees       .
c     .           ng = number of integration points                   .
c     .                (theta points for random orientation)          .
c     .                                                               .
c     .  the a,b, and t matrices are calculated as in program t1.for  .
c     .  arrays are set for nrank = 25 and ntheta = 100 maximum       .
c     .                                                               .
c     .  program is currently configured for mirror symmetric         .
c     .    particles.  program modification for particles without     .
c     .    mirror symmetry is achieved by adding and deleting the     .
c     .    statements noted in columns 73-80                          .
c     .................................................................
      complex a,b,c,dcn,cm,h,bk,hbk,bbk,hankel(26),bslcmp(26),sa,sb
     1,ckpr,ab1,ab2,fg1(50),fg2(50),hank(26,100),bslc(26,100),ci,c1,c2
     2,abg1(50),abg2(50)
      common dtr,rtd,pi
      common /mtxcom/ nrank,nranki,a(50,50),b(50,50)
      common /cmvcom/ nm,kmv,cmv,twm,prodm
      common /thtcom/ theta,sinth,costh,x
      common /srtcom/ ab1(50,26),ab2(50,26),cmx(25,26),anginc,nl,nh
      dimension wt(100),asc(100),wtt(25),tang(25),pnmllg(26)
      dimension acans(25,2),clrtot(50),clrmtx(10000)
      equivalence (a,clrmtx),(acans,clrtot)
      logical first
c     ...........................
c     .  set program constants  .
c     ...........................
      dtr =.017453292519943
      rtd = 57.2957795131
      pi = 3.14159265358979
      pi2 = pi/2.0
      ci = (0.0,1.0)
      open(unit=9,file='t7.dat')
      rewind 9
      write(6,230)
      write(6,*) 'enter nrank, ntheta'
      read(5,*) nrank,ntheta
      nranki = nrank+1
      nr2 = 2*nrank
      nm = nrank
      write(6,240)
      write(6,*) 'enter aovrb, mr, mi'
      read(5,*) aovrb,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      dcn = cm**2
      write(6,250)
      write(6,*) 'enter start ka, end ka, npnts, case'
      read(5,*) cnk,cnke,npnts,ic
      dk = 0.0
      if(npnts.ne.1) dk = (cnke-cnk)/real(npnts-1)
c     ...........................................
c     .  set the do 180 loop indices for all m  .
c     ...........................................
      nl = 1
      nh = nm
c     ..............................................
c     .  set the parameters for fixed orientation  .
c     ..............................................
      if(ic.eq.1) then
        write(6,260)
        write(6,*) 'enter anginc'
        read(5,*) anginc
        if(anginc.eq.0.0.or.anginc.eq.180.0) then
          nl = 2
          nh = 2
        end if
        anginc = anginc*dtr
c     ...............................................
c     .  set the parameters for random orientation  .
c     ...............................................
      else
        write(6,270)
        write(6,*) 'enter number of theta orientations'
        read(5,*) ng
c     .........................................................
c     .  set the integration points and weighting values for  .
c     .    integration over random orientation                .
c     .........................................................
        call gauss(wtt,asc,ng,0.0,1.0)
        do 10 i = 1,ng
          tang(i) = acos(asc(i))
10      continue
        anginc = tang(1)
      end if
c     .............................................................
c     .  call subroutine start to calculate the incident field    .
c     .    coefficients and normalization factor for all m        .
c     .    anginc = angle of incidence for fixed orientation and  .
c     .    the first angle of incidence for random orientation    .
c     .............................................................
      call start
c     .....................................................
c     .  set the integration points and weighting values  .
c     .....................................................
      call gauss(wt,asc,ntheta,0.0,pi2)                                 delete
c      call gauss(wt,asc,ntheta,0.0,pi)                                  add
c     ..........................................
c     .  enter a loop for each size parameter  .
c     ..........................................
      write(6,*) 'data point'
      do 210 nc = 1,npnts
        x = cnk+real(nc-1)*dk
        write(6,*) nc
c     ...................................
c     .  clear the result accumulators  .
c     ...................................
        f1 = 0.0
        f2 = 0.0
        if(ic.eq.2) then
          do 20 i = 1,50
            clrtot(i) = 0.0
20        continue
        end if
        first = .true.
c     ............................................
c     .  enter a loop for each azimuthal mode m  .
c     ............................................
        do 180 im = nl,nh 
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
          kmv = im-1
          if(nm.eq.1) kmv = 1
          cmv = real(kmv)
          cm2 = cmv**2 
          prodm = 1.0 
          if(kmv.eq.0) then
            em = 1.0
          else
            em = 2.0
            quanm = cmv
            do 30 i = 1,kmv
              quanm = quanm+1.0
              prodm = quanm*prodm/2.0
30          continue
          end if
          qem = 2.0/em 
          twm = 2.0*cmv 
c     ...................................................
c     .  set indices for matrix compression when n < m  .
c     .    note: useful only when m > 1                 .
c     ...................................................
          ij = kmv-1
          if(ij.lt.0) ij = 0
          ijt = 2*ij
          ns = nrank-ij
          ns2 = 2*ns
c     ............................................
c     .  initialize all matrix elements to zero  .
c     ............................................
          do 40 i = 1,10000
            clrmtx(i) = 0.0 
40        continue 
c     ................................................
c     .  enter a loop to integrate over the surface  .
c     .    (theta is the integration variable)       .
c     ................................................
          do 90 ithta = 1,ntheta 
            theta = asc(ithta) 
            sinth = sin(theta) 
            costh = cos(theta) 
c     .................................................
c     .  calculate the associated Legendre functions  .
c     .    at each integration point                  .
c     .................................................
            call genlgp(pnmllg,nranki)
c     ......................................
c     .  calculate kr and it's derivative  .
c     .    at each integration point       .
c     ...................................... 
c     .......................................................
c     .  current shape descriptor is for a spheroid:        .
c     .    a < b : oblate, a = 0 : sphere, a > b : prolate  .
c     .    dimension = 2a (along the symmetry axis) x 2b    .
c     .......................................................
            qb = 1.0/sqrt(costh**2+(aovrb*sinth)**2)
            ckr = x*qb                                                  eq 3.71
            dckr = -x*costh*sinth*(aovrb**2-1.0)*qb**3                  eq 3.72
c     ........................................................
c     .  calculate the Hankel functions (real argument) and  .
c     .    Bessel functions (complex argument) at each       .
c     .    integration point and store for later use         .
c     ........................................................
            if(im.eq.nl) then
              call besh(ckr,hankel,nranki)
              ckpr = cm*ckr
              call besj(ckpr,bslcmp,nranki)
              do 50 i = 1,nranki
                hank(i,ithta) = hankel(i)
                bslc(i,ithta) = bslcmp(i)
50            continue
            else
              do 60 i = 1,nranki
                hankel(i) = hank(i,ithta)
                bslcmp(i) = bslc(i,ithta)
60            continue
            end if
            d = dckr*sinth
            wtsin = 2.0*wt(ithta)*sinth                                 delete
c            wtsin = wt(ithta)*sinth                                     add
c     .............................................
c     .  enter a loop for each row of the matrix  .
c     .............................................
            do 80 irow = 1,nrank 
              if(irow.le.ij) go to 80
              irow1 = irow+nrank 
              crow = real(irow) 
              crowm = cmv+crow 
              crow1 = crow+1.0 
              br = real(hankel(irow))/real(hankel(irow+1)) 
              h = hankel(irow)/hankel(irow+1) 
c     ................................................
c     .  enter a loop for each column of the matrix  .
c     ................................................
              do 70 icol = 1,nrank 
                if(icol.le.ij) go to 70
                icol1 = icol+nrank 
                ccol = real(icol)
                ccolm = cmv+ccol
                ccol1 = ccol+1.0 
c     .....................................
c     .  calculate variable combinations  . 
c     .....................................
                crij = crow+ccol 
                crssij = crow*ccol 
                cmcrco = cm2+qem*crssij*costh**2 
                pnr0c0 = pnmllg(irow)*pnmllg(icol) 
                pnr0c1 = pnmllg(irow)*pnmllg(icol+1) 
                pnr1c0 = pnmllg(irow+1)*pnmllg(icol) 
                pnr1c1 = pnmllg(irow+1)*pnmllg(icol+1) 
                b1a = crow*costh*pnr1c1-crowm*pnr0c1 
                b1b = ccol*costh*pnr1c1-ccolm*pnr1c0 
                bk = cm*bslcmp(icol)/bslcmp(icol+1)
                bbk = real(hankel(irow+1))*bslcmp(icol+1)*wtsin
                hbk = hankel(irow+1)*bslcmp(icol+1)*wtsin
c     .....................................................
c     .  calculate the [A]' and  [B]' submatrices, e.g.,  .
c     .            _                             _        .
c     .           |               |               |       .
c     .           |             ,               , |       .
c     .           | (K + cm * J)  |-(I + cm * L)  |       .
c     .     ,     |                               |       .
c     .  [A]  =   |- - - - - - - -|- - - - - - - -|       .
c     .           |             ,               , |       .
c     .           | (L + cm * I)  | (J + cm * K)  |       .
c     .           |                               |       .
c     .           |_              |              _|       .
c     .                                                   .
c     .....................................................
c     ...............................................................
c     .  if (irow+icol) is even then the I and L elements are zero  .
c     .  if (irow+icol) is odd  then the J and K elements are zero  .
c     ...............................................................
      if(mod((irow+icol),2).ne.0) then                                  delete
c     ....................................................
c     .  increment the -(I + cm * L)' submatrix element  .
c     ....................................................
        if(kmv.ne.0) then 
          b1 = b1a+b1b 
          sa = pnr1c1*
     1         (crow*crow1*bk+ccol*ccol1*h-crssij*(crij+2.0)/ckr)*d
          sa = (ckr*(1.0+h*bk)-ccol*h-crow*bk+crssij/ckr)*b1*ckr+sa
          a(icol-ij,irow1-ijt) = a(icol-ij,irow1-ijt)+cmv*sa*hbk        eq 3.23
          sb = sa+(br-h)*(ccol*ccol1*pnr1c1*d+(bk*ckr-ccol)*b1*ckr)
          b(icol-ij,irow1-ijt) = b(icol-ij,irow1-ijt)+cmv*sb*bbk
c     ...................................................
c     .  increment the (L + cm * I)' submatrix element  .
c     ...................................................
          c = (dcn-1.0)*b1*ckr**2
          a(icol1-ijt,irow-ij) = a(icol1-ijt,irow-ij)-cmv*(sa+c)*hbk/cm eq 3.24
          b(icol1-ijt,irow-ij) = b(icol1-ijt,irow-ij)-cmv*(sb+c)*bbk/cm
        end if
      else                                                              delete
c     ...................................................
c     .  increment the (J + cm * K)' submatrix element  .
c     ...................................................
        a12 = cmcrco*pnr1c1-qem*(crow*ccolm*costh*pnr1c0+
     1  ccol*crowm*costh*pnr0c1-crowm*ccolm*pnr0c0)
        a22 = a12*ckr**2
        b1a = ccol*ccol1*b1a
        b1b = crow*crow1*b1b
        sa = (ckr*(bk-dcn*h)+dcn*crow-ccol)*a12*ckr+(b1a-dcn*b1b)*qem*d
        a(icol1-ijt,irow1-ijt) = a(icol1-ijt,irow1-ijt)+sa*hbk/cm       eq 3.25
        b(icol1-ijt,irow1-ijt) = b(icol1-ijt,irow1-ijt)
     1                           +(sa-(br-h)*dcn*a22)*bbk/cm
c     ...................................................
c     .  increment the (K + cm * J)' submatrix element  .
c     ...................................................
        sa = (ckr*(bk-h)+crow-ccol)*a12*ckr+(b1a-b1b)*qem*d
        a(icol-ij,irow-ij) = a(icol-ij,irow-ij)+sa*hbk                  eq 3.26
        b(icol-ij,irow-ij) = b(icol-ij,irow-ij)+(sa-(br-h)*a22)*bbk
      end if                                                            delete
70            continue 
80          continue
90        continue
          call prcssm(ns2)
c     .............................................
c     .  [T]' is stored in the b(*,*) array       .
c     .    reverse the indices to obtain a        .
c     .    multiplication by [T]                  .
c     .  the scattered field coefficients = -[T]  .
c     .    times the incident field coefficients  .
c     .  fg2(ns+1) to fg2(ns) is -g(omn) (3.12b)  .
c     .............................................
          do 110 i = 1,ns2
            fg1(i) = 0.0
            fg2(i) = 0.0
            do 100 j = 1,ns2
              fg1(i) = fg1(i)-b(j,i)*ab1(j,im)                          eq 3.12a
              fg2(i) = fg2(i)-b(j,i)*ab2(j,im)                          eq 3.12b
100         continue
110       continue
c     .......................................................
c     .  calculate the normalized scattering cross section  .
c     .    for angle of incidence = anginc                  .
c     .  results are accumulated for each azimuthal mode m  .
c     .  f1 = parallel and f2 = perpendicular polarization  .
c     .......................................................
          do 120 n = 1,ns
            np = n+ns
            f1 = f1+(abs(fg1(n))**2+abs(fg1(np))**2)*cmx(n,im)          eq 3.53
            f2 = f2+(abs(fg2(n))**2+abs(fg2(np))**2)*cmx(n,im)          eq 3.53
120       continue
c     .........................................................
c     .  if random orientation then calculate the scattering  .
c     .    cross section for additional angles of incidence   .
c     .........................................................
          if(ic.eq.2) then
c     ...............................................
c     .  calculate the incident field coefficients  .
c     ...............................................
            do 170 ig = 2,ng
              theta = tang(ig)
              sinth = sin(theta)
              costh = cos(theta)
              call genlgp(pnmllg,nranki)
              do 130 n = 1,nrank
                if(n.le.ij) go to 130
                np = n+nrank
                cn = real(n)
                n1 = n+1 
                c1 = 4.0*ci**n 
                c2 = 4.0*ci**n1 
                p1 = cn*costh*pnmllg(n1)-(cn+cmv)*pnmllg(n) 
                p2 = cmv*pnmllg(n1) 
                abg1(n-ij) = c1*p2 
                abg1(np-ijt) = -c2*p1 
                abg2(n-ij) = -c1*p1 
                abg2(np-ijt) = c2*p2 
130           continue
c     .............................................
c     .  the scattered field coefficients = -[T]  .
c     .    times the incident field coefficients  .
c     .  fg2(ns+1) to fg2(ns2) is -g(omn)         .
c     .............................................
              do 150 i = 1,ns2
                fg1(i) = 0.0
                fg2(i) = 0.0
                do 140 j = 1,ns2
                  fg1(i) = fg1(i)-b(j,i)*abg1(j)                        eq 3.12a
                  fg2(i) = fg2(i)-b(j,i)*abg2(j)                        eq 3.12b
140             continue
150           continue
c     .......................................................
c     .  calculate the normalized scattering cross section  .
c     .    for all angles of incidence                      .
c     .  results are accumulated for each azimuthal mode m  .
c     .  acans(ig,1) = parallel polarization                .
c     .  acans(ig,2) = perpendicular polarization           .
c     .......................................................
              do 160 n = 1,ns
                np = n+ns
                acans(ig,1) = acans(ig,1)
     1          +(abs(fg1(n))**2+abs(fg1(np))**2)*cmx(n,im)             eq 3.53
c     ....................................
c     .  fg2(np) is -g(omn) per (3.12b)  .
c     ....................................
                acans(ig,2) = acans(ig,2)
     1          +(abs(fg2(n))**2+abs(fg2(np))**2)*cmx(n,im)             eq 3.53
160           continue
170         continue
          end if
c     .......................................
c     .  test the scattering cross section  .
c     .    for convergence at anginc        .
c     .......................................
          nconv = 0
          if(.not.first) then
            if(abs(1.0-oldf1/f1).le.1.0e-02) nconv = nconv+1
            if(abs(1.0-oldf2/f2).le.1.0e-02) nconv = nconv+1
          end if
          if(nconv.eq.2) go to 190
          oldf1 = f1
          oldf2 = f2
          first = .false.
180     continue
c     ................................................
c     .  normalize the scattering cross section for  .
c     .    fixed orientation and write the results   .
c     ................................................
190     if(ic.eq.1) then
          f1 = f1/x**2
          f2 = f2/x**2
          write(9,220) x,f1,f2
c     ................................................
c     .  integrate the scattering cross section to   .
c     .    obtain the result for random orientation  .
c     ................................................
        else
          scat = wtt(1)*(f1+f2)
          do 200 ig = 2,ng
            scat = scat+wtt(ig)*(acans(ig,1)+acans(ig,2))               eq 3.54b
200       continue
c     ................................................
c     .  normalize the scattering cross section for  .
c     .    random orientation and write the results  .
c     ................................................
          scat = scat/(2.0*x**2)
          write(9,220) x,scat
        end if
210   continue
      close(unit=9)
      stop
220   format(3e14.6)
230   format('................................................',/,
     1       '.  calculate scattering cross section vs size  .',/,
     2       '.    parameter - output is written to t7.dat   .',/,
     3       '................................................',//,
     4       'nrank: number of terms',/,
     5       'ntheta: number of integration steps',/)
240   format(/,'aovrb: a/b ratio',/,
     1       'index of refraction: real,imaginary (+ for absorption)',/)
250   format(/,'starting ka: starting size parameter',/,
     1         'ending ka: ending size parameter',/,
     2         'npnts: number of size parameter values',/,
     3         'case: orientation fixed (1) or random (2)',/)
260   format(/,'fixed orientation',/,
     1         'output is x, parallel polar., perpendicular polar.',//,
     2         'anginc: angle of incidence, 0 to 180 degrees',/)
270   format(/,'random orientation',/,
     1         'output is x, scattering cross section',//,
     2         'theta orientations: number of integration steps',/)
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
      subroutine besh(x,hankel,nc)
c     ...................................................
c     .  calculate Hankel functions                     .
c     .  bj = Bessel function of the first kind         .
c     .  by = Bessel function of the second kind        .
c     .  x = real argument                              . 
c     .  nc = number of orders (0 to nc-1)              .
c     .  the order of the functions is incremented by   .
c     .    one in the bj(*),by(*) and hankel(*) arrays  .
c     .                                                 .
c     .  arrays are set for nc = 26 maximum             .
c     ...................................................
      complex hankel(nc)
      dimension bj(26),by(26),t(3)
c     ................................................
c     .  by(*) calculation - obtain the zeroeth and  .
c     .                      first order functions   .
c     ................................................
      a = sin(x)/x                                                      eq 3.73
      by(1) = -cos(x)/x                                                 eq 3.74
      by(2) = by(1)/x-a                                                 eq 3.75
c     ...........................................................
c     .  obtain the higher order functions by upward recursion  .
c     ...........................................................
      do 10 n = 3,nc
        rn = real(n-2)
        by(n) = (2.0*rn+1.0)*by(n-1)/x-by(n-2)
10    continue
c     ................................................
c     .  bj(*) calculation - set the starting order  .
c     .                      for downward recursion  .
c     ................................................
      nst = nc+int((101.0+x)**.5)
c     ....................................................
c     .  the t(*) array is used to recur down to the     .
c     .    two highest order functions that are needed   .
c     .  set starting values for the two highest orders  .
c     .    nst and nst-1                                 .
c     ....................................................
      t(3) = 0.0
      t(2) = 1.0e-35
c     ...................................................
c     .  recur downward to obtain orders nc-1 and nc-2  .
c     ...................................................
      do 20 i = nst-1,nc-1,-1
        ri = real(i)
        t(1) = (2.0*ri+1.0)*t(2)/x-t(3)
        t(3) = t(2)
        t(2) = t(1)
20    continue
c     ...............................................
c     .  continue downward recursion to order zero  .
c     ...............................................
      bj(nc) = t(3)
      bj(nc-1) = t(2)
      do 30 i = nc-2,1,-1
        ri = real(i)
        bj(i) = (2.0*ri+1.0)*bj(i+1)/x-bj(i+2)
30    continue                
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
      alpha = a/bj(1)
      do 40 k = 1,nc
        hankel(k) = cmplx(bj(k)*alpha,by(k))
40    continue
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
      nst =int(x+4.05*x**.3333+2.0+(101.0+x)**.5)
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
      subroutine prcssm(n) 
c     ...............................................................
c     .                      -1                                     .
c     .  calculate [T]' = [A]' * [B]' using Gauss-Jordan reduction  .
c     ...............................................................
      complex a,b,aijmax,arat
      common /mtxcom/ nr,nri,a(50,50),b(50,50)
      dimension ls(50)
c     ............................
c     .  start reduction of [a]  .
c     ............................
      do 60 i = 1,n 
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
c     ....................................................
c     .  normalize the ith row of [a] and [b] by aijmax  .
c     ....................................................
        do 20 j = 1,n 
          a(i,j) = a(i,j)/aijmax 
          b(i,j) = b(i,j)/aijmax 
20      continue 
c     .......................................................
c     .  use row transformations to obtain zeros above and  .
c     .    below the jmax element of the ith row of [a] -   .
c     .    apply the same row transformations to [b]        .
c     .......................................................
        do 50 k = 1,n
          if(k.ne.i) then
            arat = -a(k,jmax) 
            do 30 j = 1,n
              if(abs(a(i,j)).gt.0.0) then
                a(k,j) = arat*a(i,j)+a(k,j) 
              end if
30          continue 
            a(k,jmax) = 0.0 
            do 40 j=1,n
              if(abs(b(i,j)).gt.0.0) then
                b(k,j) = arat*b(i,j)+b(k,j) 
              end if
40          continue 
          end if
50      continue 
c     ........................................................
c     .  store row counter (i) in array ls(*), such that     .
c     .    ls(*) contains the location of the pivot (unity)  .
c     .    element of each column (after reduction)          .
c     ........................................................
        ls(jmax) = i
60    continue 
c     ....................................................
c     .  the reduction of [a] is complete - perform      .
c     .    row interchanges as indicated in array ls(*)  .
c     ....................................................
      do 90 i = 1,n 
        k = i 
c     .........................................................
c     .  put the integer value in ls(k) into k                .
c     .                                                       .
c     .  if k is less than i, then that row has already been  .
c     .    involved in an interchange so iterate k = ls(k)    .
c     .    until a value of k greater than i (corresponding   . 
c     .    to a row stored above the ith row) is obtained     .
c     .........................................................
70      k = ls(k)
        if(k.lt.i) go to 70
        if(k.gt.0) then
          do 80 j=1,n 
            arat = b(i,j) 
            b(i,j) = b(k,j) 
            b(k,j) = arat 
80        continue 
        end if
90    continue
c     ........................................
c     .  [T]' is stored in the b(*,*) array  .
c     ........................................
      return
      end 
      subroutine start
      complex a,b,ab1,ab2,ci,c1,c2
      common dtr,rtd,pi
      common /mtxcom/ nrank,nranki,a(50,50),b(50,50)
      common /cmvcom/ nm,kmv,cmv,twm,prodm
      common /thtcom/ theta,sinth,costh,x
      common /srtcom/ ab1(50,26),ab2(50,26),cmx(25,26),anginc,nl,nh
      dimension pnmllg(26)
      ci = (0.0,1.0)
      theta = anginc
      sinth = sin(theta)
      costh = cos(theta)
      if(theta.eq.pi) then
        theta = 0.0
        costh = -1.0
      end if
      do 50 im = nl,nh 
c     ...............................
c     .  set m-dependent variables  .
c     ...............................
        kmv = im-1
        if(nm.eq.1) kmv = 1
        cmv = real(kmv)
        prodm = 1.0 
        if(kmv.eq.0) then
          em = 1.0
        else
          em = 2.0
          quanm = cmv
          do 10 i = 1,kmv
            quanm = quanm+1.0
            prodm = quanm*prodm/2.0
10        continue
        end if
        twm = 2.0*cmv 
c     ...................................................
c     .  set indices for matrix compression when n < m  .
c     .    note: useful only when m > 1                 .
c     ...................................................
        ij = kmv-1
        if(ij.lt.0) ij = 0
        ijt = 2*ij
c     ..................................................
c     .  calculate the incident field coefficients:    .
c     .    ab1(*) = theta polarization                 .
c     .    ab2(*) = phi polarization                   .
c     .  note: ab1(*) and ab2(*) used in scattered     .
c     .        field programs are j times ab1(*) and   .
c     .        ab2(*) used in internal field programs  .            
c     ..................................................
        call genlgp(pnmllg,nranki)
        do 20 n = 1,nrank 
          if(n.le.ij) go to 20
          np = n+nrank 
          cn = real(n)
          n1 = n+1 
          c1 = 4.0*ci**n 
          c2 = 4.0*ci**n1 
          p1 = cn*costh*pnmllg(n1)-(cn+cmv)*pnmllg(n) 
          p2 = cmv*pnmllg(n1) 
          ab1(n-ij,im) = c1*p2 
          ab1(np-ijt,im) = -c2*p1 
          ab2(n-ij,im) = -c1*p1 
          ab2(np-ijt,im) = c2*p2 
20      continue 
c     ........................................
c     .  calculate the normalization factor  . 
c     ........................................
        do 40 irow = 1,nrank
          if(irow.le.ij) go to 40
          crow = real(irow)
          if(kmv.eq.0) then 
            cmx(irow-ij,im) = (2.0*crow+1.0)/(4.0*crow*(crow+1.0))      eq 3.2
          else
            if(irow.lt.kmv) then
              cmx(irow-ij,im) = 1.0
            else
              ib = irow-kmv+1 
              ie = irow+kmv 
              fctki = 1.0
              fprod = real(ib)
              do 30 l = ib,ie 
                fctki = fctki*fprod 
                fprod = fprod+1.0 
30            continue 
              cmx(irow-ij,im) =  em*(2.0*crow+1.0)/
     1                           (4.0*crow*(crow+1.0)*fctki)            eq 3.2
            end if
          end if
40      continue
50    continue
      return
      end
