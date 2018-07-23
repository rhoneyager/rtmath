      program T1
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .................................................................
c     .  calculate the scattering by axisymmetric dielectric objects  .
c     .  convergence test over three parameters:                      .
c     .    nrank : number of terms (matrix order)                     .
c     .    ntheta: number of integration steps                        .
c     .    nm    : number of azimuthal modes m                        .
c     .  inputs:  nrank  = matrix order                               .
c     .           ntheta = integration steps                          .
c     .           ic = convergence case                               .
c     .                ntheta ( = 0 ), nrank ( = 1 ), or nm ( = 2 )   .
c     .           x = size parameter (ka)                             .
c     .           aovrb = a/b ratio                                   .
c     .           cm = complex index of refraction, (real,imag)       .
c     .                (imag is positive for absorption)              .
c     .                                                               .
c     .  arrays are set for nrank = 25 and ntheta = 100 maximum       .
c     .                           -1         -1                       .
c     .  calculate [T] = [B] * [A]   =  { [A]' * [B]'}'               .
c     .                                                               .
c     .    use transposed matrices to permit the efficient            .
c     .       -1                                      -1              .
c     .     [ ]  * [ ]  operation rather than [ ] * [ ]               .
c     .                                                               .
c     .    (1)  obtain the transposed matrices [A]' and [B]'          .
c     .                             -1                                .
c     .    (2)  calculate [T]' = [A]' * [B]'                          .
c     .                                                               .
c     .    (3)  transpose [T]' to obtain [T]                          .
c     .                                                               .     
c     .  two simplifications are incorporated in the program to       .
c     .    speed up the computation:                                  .
c     .                                                               .
c     .  (1)  for mirror-symmetric objects, e.g., spheroids,          .
c     .       the contribution to the integrals over the particle     .
c     .       surface from zero to 90 degrees is cancelled by a       .
c     .       contribution of opposite sign from 90 to 180            .
c     .       degrees for half of the matrix elements and is          .
c     .       doubled by a contribution of the same sign from         .
c     .       90 to 180 degrees for the other half of the             .
c     .       matrix elements.  the program is currently              .
c     .       configured for mirror-symmetric particles.              .
c     .       program modification for particles without mirror       .
c     .       symmetry is achieved by adding and deleting the         .
c     .       statements noted in columns 73-80                       .
c     .                                                               .
c     .  (2)  all matrix elements and incident and scattered          .
c     .       field coefficients are zero when n < m by virtue        .
c     .       of the behavior of the associated Legendre              .
c     .       functions.  all matrices and coefficient arrays         .
c     .       are compressed to take advantage of this behavior       .
c     .................................................................
      complex a,b,c,dcn,cm,h,bk,hbk,bbk,hankel(26),bslcmp(26),sa,sb,
     1ckpr,aa(50,50),bb(50,50)
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,a(50,50),b(50,50),cmxnrm(25)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,aovrb,cmr,cmi,ic
      dimension clrmtx(10000),wt(100),asc(100),pnmllg(26)
      equivalence (a,clrmtx) 
c     ...........................
c     .  set program constants  .
c     ...........................
      dtr =.017453292519943 
      rtd = 57.2957795131 
      pi = 3.14159265358979 
      pi2 = pi/2.0
      write(6,200)
      write(6,*) 'enter nrank, ntheta, case'
      read(5,*) nrank,ntheta,ic
      nranki = nrank+1
      nr2 = 2*nrank
      write(6,210)
      write(6,*) 'enter x, aovrb, mr, mi'
      read(5,*) x,aovrb,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      dcn = cm**2
c     .................................................................
c     .  utheta and uphi are the amplitudes of the incident field in  .
c     .    the theta (parallel) and phi (perpendicular) directions    .
c     .................................................................
      utheta = 1.0/sqrt(2.0)
      uphi = 1.0/sqrt(2.0)
c     .................................................................
c     .  convergence over m (ic = 2) requires nrank azimuthal modes,  .
c     .    a nonsymmetric orientation, and a file for the matrices    .
c     .................................................................
      mtxsav = 0
      if(ic.eq.2) then
        nm = nrank
        angint = pi/4.0
        anginp = pi/4.0
        write(6,220)
        write(6,*) 'enter 1 (save [T]) or 2 (save [A])'
        read(5,*) mtxsav
        if(mtxsav.eq.1) then
          open(unit=11,file='t',form='unformatted')
        else
          open(unit=11,file='a',form='unformatted')
        end if
        rewind 11
c     .................................................................
c     .  convergence over ntheta (ic = 0) or nrank (ic = 1) requires  .
c     .    one azimuthal mode (m = 1) and a symmetric orientation     .
c     .................................................................
      else
        nm = 1
        angint = 0.0
        anginp = 0.0
      end if
      write(6,230) nm,nrank,ntheta,ic
      write(6,240) x,aovrb,cmr,cmi
      angtpr = angint*rtd
      angppr = anginp*rtd
      write(6,250) utheta,uphi,angtpr,angppr
c     ...............................................
c     .  convergence over ntheta (ic = 0) requires  .
c     .    solutions for two values of ntheta       .
c     ...............................................
      it = 1
      if(ic.eq.0) it = 2
      do 160 icon = 1,it
c     .....................................................
c     .  set the integration points and weighting values  .
c     .....................................................
        call gauss(wt,asc,ntheta,0.0,pi2)                               delete
c        call gauss(wt,asc,ntheta,0.0,pi)                                add
c     ............................................
c     .  enter a loop for each azimuthal mode m  .
c     ............................................
        do 150 im = 1,nm 
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
            do 10 i = 1,kmv
              quanm = quanm+1.0
              prodm = quanm*prodm/2.0
10          continue
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
          do 20 i = 1,10000
            clrmtx(i) = 0.0 
20        continue 
c     ................................................
c     .  enter a loop to integrate over the surface  .
c     .    (theta is the integration variable)       .
c     ................................................
          do 50 ithta = 1,ntheta 
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
c     .    integration point                                 .
c     ........................................................
            call besh(ckr,hankel,nranki)
            ckpr = cm*ckr
            call besj(ckpr,bslcmp,nranki)
            d = dckr*sinth
            wtsin = 2.0*wt(ithta)*sinth                                 delete
c            wtsin = wt(ithta)*sinth                                     add
c     .............................................
c     .  enter a loop for each row of the matrix  .
c     .............................................
            do 40 irow = 1,nrank 
              if(irow.le.ij) go to 40
              irow1 = irow+nrank 
              crow = real(irow) 
              crowm = cmv+crow 
              crow1 = crow+1.0 
              br = real(hankel(irow))/real(hankel(irow+1)) 
              h = hankel(irow)/hankel(irow+1) 
c     ................................................
c     .  enter a loop for each column of the matrix  .
c     ................................................
              do 30 icol = 1,nrank 
                if(icol.le.ij) go to 30
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
c     .  simplification for mirror-symmetric particles:             .
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
30            continue 
40          continue
50        continue
c     ........................................
c     .  calculate the normalization factor  . 
c     ........................................
          do 70 irow = 1,nrank
            if(irow.le.ij) go to 70
            crow = real(irow)
            if(kmv.eq.0) then 
              cmxnrm(irow-ij) = (2.0*crow+1.0)/(4.0*crow*(crow+1.0))    eq 3.2
            else
              if(irow.lt.kmv) then
                cmxnrm(irow-ij) = 1.0
              else
                ib = irow-kmv+1 
                ie = irow+kmv 
                fctki = 1.0
                fprod = real(ib)
                do 60 l = ib,ie 
                  fctki = fctki*fprod 
                  fprod = fprod+1.0 
60              continue 
                  cmxnrm(irow-ij) =  em*(2.0*crow+1.0)/
     1                               (4.0*crow*(crow+1.0)*fctki)        eq 3.2
              end if
            end if
70        continue
c     ...................................................
c     .  convergence over nrank (ic = 1) requires that  .
c     .    the [A]'and [B]' matrices be stored for      .
c     .    a second iteration with nrank = nrank - 1    .
c     ...................................................
          if(ic.eq.1) then
            do 80 ii = 1,nr2
              do 75 jj = 1,nr2
                aa(ii,jj) = a(ii,jj)
                bb(ii,jj) = b(ii,jj)
75            continue
80          continue
          end if
c     ..................................................
c     .  for convergence over nm (ic = 2) save the     .
c     .    [A] matrix and normalization constants      .
c     .    for later use in internal field programs    .
c     .  [A]' is stored in the a(*,*) array            .
c     .  transpose and store in the aa(*,*) array      .
c     ..................................................
          if(ic.eq.2.and.mtxsav.eq.2) then
            do 90 ir = 1,ns2
              do 85 jr = 1,ns2
                aa(ir,jr) = a(jr,ir)
85            continue
90          continue
            write(11) aa
          end if
c     ..........................................
c     .                      -1                .
c     .  calculate [T]' = [A]' * [B]'          .
c     ..........................................
          call prcssm(ns2)
c     ........................................
c     .  [T]' is stored in the b(*,*) array  .
c     .    transpose [T]' to obtain [T]      .
c     ........................................
          do 100 ir = 1,ns2
            do 95 jr = 1,ns2
              a(ir,jr) = b(jr,ir)
95          continue
100       continue
c     ..................................................
c     .  for convergence over nm (ic = 2) save the     .
c     .    [T] matrix and normalization constants      .
c     .    for later use in scattering programs        .
c     .  the [T] matrix is stored in the a(*,*) array  .
c     ..................................................
          if(ic.eq.2.and.mtxsav.eq.1) write(11) a,cmxnrm
c     .......................................................
c     .  calculate the angular scattering and efficiencies  .
c     .......................................................
          call addprc
c     .......................................................
c     .  convergence over nrank (ic = 1) requires a second  .
c     .    solution with nrank = nrank-1                    .
c     .......................................................
          if(ic.eq.1) then
c     .................................................
c     .  restore the original [A]' and [B]' matrices  .
c     .................................................
            do 110 ii = 1,nr2
              do 105  jj = 1,nr2
                a(ii,jj) = aa(ii,jj)
                b(ii,jj) = bb(ii,jj)
105           continue
110         continue
c     ..............................................
c     .  delete the last row and column of each    .
c     .    submatrix to reduce nrank to nrank - 1  .
c     ..............................................
            nr2m = nr2-1
            nr2 = 2*(nrank-1)
            do 120 ir = 1,nr2m
              do 115 jr = nrank,nr2
                a(ir,jr) = a(ir,jr+1)
                b(ir,jr) = b(ir,jr+1)
115           continue
120         continue
            do 130 jr = 1,nr2
              do 125 ir = nrank,nr2
                a(ir,jr) = a(ir+1,jr)
                b(ir,jr) = b(ir+1,jr)
125           continue
130         continue
            nranki = nrank
            nrank = nrank-1
            write(6,230) nm,nrank,ntheta,ic
c     ..........................................
c     .                      -1                .
c     .  calculate [T]' = [A]' * [B]'          .
c     ..........................................
            call prcssm(nr2)
c     ........................................
c     .  [T]' is stored in the b(*,*) array  .
c     .    transpose [T]' to obtain [T]      .
c     ........................................
            do 140 ir = 1,nr2
              do 135 jr = 1,nr2
                a(ir,jr) = b(jr,ir)
135           continue
140         continue
c     .......................................................
c     .  calculate the angular scattering and efficiencies  .
c     .......................................................
            call addprc
          end if
150     continue 
c     ...............................................
c     .  convergence over ntheta (ic = 0) requires  .
c     .    a solution for a second value of ntheta  .
c     ...............................................
        if(ic.eq.0.and.icon.eq.1) then
          ntheta = ntheta+4
          write(6,230) nm,nrank,ntheta,ic
        end if
160   continue
      stop
200   format('..........................................',/,
     1       '.  convergence test - save the matrices  .',/,
     2       '.    [T] is written to file ''t''          .',/,
     3       '.    [A] is written to file ''a''          .',/,
     4       '.    parameters are in file ''case''       .',/,
     5       '..........................................',//,
     6       'nrank: number of terms',/,
     7       'ntheta: number of integration steps',/,
     8       'convergence case: ntheta (0), nrank (1), nm (2)',/)
210   format(/,'x: size parameter',/,
     1       'aovrb: a/b ratio',/,
     2       'index of refraction: real,imaginary (+ for absorption)',/)
220   format(/,'save [T] or [A] matrix',/)
230   format(/,'nm',i8,3x,'nrank',i6,3x,'ntheta',i8,3x,'ic',i12)
240   format('ka',f8.3,3x,'aovrb',f6.3,3x,'mr',f12.3,3x,'mi',f12.3)
250   format('utht',f6.3,3x,'uphi',f7.3,3x,'angint',f8.3,3x,
     1'anginp',f8.3)
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
      nst = int(x+4.05*x**.3333+2.0+(101.0+x)**.5)
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
      common /mtxcom/ nr,nri,a(50,50),b(50,50),cmxnrm(25)
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
            do 40 j = 1,n
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
      subroutine genang(n,tang,pang,cosb,sinb) 
c     ...............................................................
c     .  calculate scattering angles in the particle frame corres-  .
c     .    ponding to scattering angles in the laboratory frame     .
c     .  calculate the elements of the transform matrix for the     .
c     .    polarization vectors                                     .
c     ...............................................................
      common dtr,rtd,pi 
      common /uvccom/ angint,anginp,utheta,uphi,aovrb,cmr,cmi,ic
      dimension tang(n),pang(n),cosb(n),sinb(n)
c     ..............................................................
c     .  dltang calculation assumes scattering over 180.0 degrees  .
c     ..............................................................
      dltang = pi/real(n-1)
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
        ps = 0.0
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
c     ...................................................
c     .  convergence test the differential scattering   .
c     .    cross section and calculate the scattering,  .
c     .    absorption and extinction efficiencies       .
c     .  calculations are performed in the particle     .
c     .    frame and written in the laboratory frame    .
c     ...................................................
      complex tmat,b,ci,c1,c2,cim,ab1(50),ab2(50),
     1acans(10,2),fg1(50),fg2(50)
      common dtr,rtd,pi 
      common /mtxcom/ nrank,nranki,tmat(50,50),b(50,50),cmxnrm(25)
      common /cmvcom/ nm,kmv,cmv,twm,prodm 
      common /thtcom/ theta,sinth,costh,x
      common /uvccom/ angint,anginp,utheta,uphi,aovrb,cmr,cmi,ic
      dimension tang(10),pang(10),cosb(10),sinb(10),pnmllg(26)
      dimension oldh(10),oldv(10),clrtot(40)
      equivalence (acans,clrtot)
      logical first 
      data first/.true./ 
c     ...............................................................
c     .  set constants and calculate the coordinate transformation  .
c     .    variables the first time the subroutine is entered       .
c     ...............................................................
      if(first) then
        ci = (0.0,1.0)
        snorm = 1.0/(pi*x**2)
        call genang(10,tang,pang,cosb,sinb)
c     .......................................................
c     .  transform the incident field polarization vector   .
c     .    from the laboratory frame to the particle frame  .
c     .    uu1 = theta component and uu2 = phi component    .
c     .......................................................
        uu1 = cosb(1)*utheta-sinb(1)*uphi                               eq 3.38a
        uu2 = sinb(1)*utheta+cosb(1)*uphi                               eq 3.38b
      end if
c     .............................................................
c     .  clear the result accumulators when ic = 0 or ic = 1 and  .
c     .    the first time the subroutine is entered for ic = 2    .
c     .............................................................           
      if(ic.le.1.or.kmv.eq.0) then
        s = 0.0
        do 10 i = 1,40
          clrtot(i) = 0.0
10      continue
        nr2 = 2*nrank
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
c     ..................................................
c     .  calculate the incident field coefficients:    .
c     .    ab1(*) = theta polarization                 .
c     .    ab2(*) = phi polarization                   .
c     .  note: ab1(*) and ab2(*) used in scattered     .
c     .        field programs are i times ab1(*) and   .
c     .        ab2(*) used in internal field programs  .            
c     ..................................................
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
      do 60 iu = 1,10 
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
c     ..........................................................
c     .  write the scattering results in the laboratory frame  .
c     .    h = horizontal (parallel) or theta polarization     .
c     .    v = vertical (perpendicular) or phi polarization    .
c     ..........................................................
      write(6,100) kmv
      nconv = 0 
      mconv = 0 
      do 70 jup = 1,10
        ang = 20.*real(jup-1) 
        h = snorm*abs(cosb(jup)*acans(jup,1)+sinb(jup)*acans(jup,2))**2 eq 3.37a
        v = snorm*abs(cosb(jup)*acans(jup,2)-sinb(jup)*acans(jup,1))**2 eq 3.37b
        write(6,110) ang,h,v
c     ...................................................
c     .  test for convergence at each scattering angle  . 
c     ...................................................
        if(.not.first) then
          if(abs(1.0-oldh(jup)/h).le.1.0e-02) nconv = nconv+1 
          if(abs(1.0-oldv(jup)/v).le.1.0e-02) mconv = mconv+1 
        end if
        oldh(jup) = h 
        oldv(jup) = v 
70    continue 
c     ..........................................................
c     .  calculate the normalized (pi*a**2) extinction (ex),   .
c     .    scattering (sc) and absorption (ab) cross sections  .
c     ..........................................................
c     ...................................................
c     .  the ex calculation can be performed in either  .
c     .    reference frame - use the particle frame     .         
c     ...................................................
      ex = (aimag(acans(1,1))*uu1+aimag(acans(1,2))*uu2)*4.0/x**2       eq 3.20
c     ..............................................................
c     .  the sc calculation is independent of the reference frame  .
c     .    results are accumulated for each azimuthal mode m       .
c     ..............................................................
      do 80 n = 1,ns 
        np = n+ns 
        s = s+(abs(fg1(n))**2+abs(fg1(np))**2
     1        +abs(fg2(n))**2+abs(fg2(np))**2)*cmxnrm(n)                eq 3.21
80    continue
        sc = s/x**2
c     ...................................................
c     .  absorption cs = extinction cs - scattering cs  .
c     ...................................................
      ab = ex-sc                                                        eq 3.22
      if(ab.lt.1.0e-08) ab = 0.0
      write(6,120) sc,ab,ex
c     ......................................
c     .  test for complete convergence of  . 
c     .    solution at 8 out of 10 angles  . 
c     .    for each polarization           .
c     .    if converged - stop             .
c     .    if not converged - return       .
c     ......................................
      if(nconv.ge.8.and.mconv.ge.8) then
        write(6,*) '*** solution has converged ***'
        if(ic.eq.2) then
          open(unit=12,file='case')
          rewind 12
          nt = kmv+1
          write(12,130) nt,nrank,x,aovrb,cmr,cmi
          close(unit=12)
          close(unit=11)
        end if
      else
        first = .false.
        return
      end if
      stop
100   format(/,12x,'***** m =',i3,' *****',/,
     1          2x,'differential scattering cross section',/,
     1          1x,'angle',9x,'parallel',8x,'perpendicular',/)
110   format(f6.2,2(5x,1pe13.4))
120   format(/,'scat xsect = ',1pe13.4,/,' abs xsect = ',
     1e13.4,/,' ext xsect = ',e13.4)
130   format(2i4,4f8.4)
      end 
