      program S5
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ............................................................
c     .  calculate the intensity (integrated over 4 pi) at a     .
c     .    particular radius for a sphere for two cases:         .
c     .  case 1: r/a .le. 1           (internal)                 .
c     .  case 2: 1 .ge. r/a .le. 1.5  (external)                 .
c     .  inputs: ic = case (internal or external)                .
c     .          x = size parameter (ka)                         .
c     .          cm = complex index of refraction, (real,imag)   .
c     .               (imag is positive for absorption)          .
c     .          npnts = number of points                        .
c     .                                                          .
c     .  dimension of arrays cf(*), dg(*), and amat(*):          .
c     .    nc = int((1.5*x)+4.05*(1.5*x)**.3333+2.0),            .
c     .      e.g., for x = 200, nc = 329                         .
c     .                                                          .
c     .  dimension of array bj(*), by(*), hkl(*): nc+2,          .
c     .    e.g., for x = 200, nc+2 = 331                         .
c     .                                                          .
c     .  dimension of array bsl(*):                              .
c     .    nc+2 = int(x+4.05*x**.3333+2.0)+2,                    .
c     .      e.g., for x = 200, nc+2 = 227                       .
c     .                                                          .
c     .  arrays are set for a maximum size parameter of 200      .
c     ............................................................
      complex ci,cm,cf(329),dg(329),hkl(331),bsl(227),dx,cmx,a,b
      common /cfcom/ cf,dg
      ci = (0.0,1.0)
      write(6,100)
      write(6,*) 'enter case, size parameter, mr, mi'
      read(5,*) ic,x,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='s5.dat')
      rewind 9
      if(ic.eq.1) then
c     .............................................
c     .  calculate internal integrated intensity  .
c     .............................................
        write(6,*) 'internal intensity - - - enter npnts ( >1 )'
        read(5,*) npnts
        cpm = real(npnts-1)
        dx = cm*x/cpm
        call sphere(x,cm,nc,ic)
        rj = 0.0
        e2 = (4./9.)*abs(dg(1))**2                                      eq 4.31
        write(9,110) rj,e2
          do 20 j = 2,npnts
          rj = real(j-1)
          cmx = rj*dx
c     ..............................................
c     .  calculate the number of terms needed for  .
c     .    convergence at each radius: nj .le. nc  .
c     ..............................................
          nj = min(int(abs(cmx)+4.05*abs(cmx)**.3333+2.0),nc)
          nj2 = nj+2
          call besj(cmx,bsl,nj2)
          e2 = 0.0
          do 10 n = 1,nj
            rn = real(n)
            rf = (rn*(rn+1.0))**2/(2.0*rn+1.0)
            e2 = e2+rf*(abs(bsl(n+1))**2*abs(cf(n))**2+((rn+1.0)*
     1           abs(bsl(n))**2+rn*abs(bsl(n+2))**2)*abs(dg(n))**2/
     2           (2.0*rn+1.0))
10        continue
          e2 = .5*e2                                                    eq 4.30
          rj = rj/cpm
          write(9,110) rj,e2
20      continue
      else
c     .............................................
c     .  calculate external integrated intensity  .
c     .............................................
        write(6,*) 'external intensity - - - enter npnts ( >1 )'
        read(5,*) npnts
        cpm = real(npnts-1)
        dtx = .5*x/cpm
        call sphere(x,cm,nc,ic)
        do 40 j = 1,npnts
          rj = real(j-1)
          xtj = x+rj*dtx
c     ..............................................
c     .  calculate the number of terms needed for  .
c     .    convergence at each radius: nj .le. nc  .
c     ..............................................
          nj = int(xtj+4.05*xtj**.3333+2.0)
          nj2 = nj+2
          call besh(xtj,hkl,nj2)
          e2 = 0.0
          do 30 n = 1,nj
            rn = real(n)
            rnum = 2.0*rn+1.0
            rden = (rn*(rn+1.0))**2
            a = 2.0*ci**n*rn*(rn+1.0)
            b = -2.0*ci**(n+1)*rn*(rn+1.0)
            bj = real(hkl(n+1))
            bjm = real(hkl(n))
            bjp = real(hkl(n+2))
            e2 = e2+rnum*bj**2+(rn+1.0)*bjm**2+rn*bjp**2
     1+abs(hkl(n+1))**2*abs(cf(n))**2*rnum/(4.0*rden)
     2+((rn+1.0)*abs(hkl(n))**2+rn*abs(hkl(n+2))**2)*abs(dg(n))**2
     3/(4.0*rden)
     4+real(a*conjg(cf(n))*bj*conjg(hkl(n+1)))*rnum/(2.0*rden)
     5+real(b*conjg(dg(n))*((rn+1.0)*bjm*conjg(hkl(n))+rn*bjp
     6*conjg(hkl(n+2))))/(2.0*rden)
30        continue
          e2 = .5*e2                                                    eq 4.32
          rj = 1.0+.5*rj/cpm
          write(9,110) rj,e2
40      continue
      end if   
      close(unit=9)
      stop
100   format('..............................................',/,
     1       '.  calculate integrated intensity vs radius  .',/,
     2       '.  output is written to s5.dat               .',/,
     3       '..............................................',//,
     4       'case: internal (1), external (2)',/,
     5       'size parameter: x',/,
     6       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(2e14.6)
      end
      subroutine sphere(x,cm,nc,ic)
c     ..............................................................
c     .  calculate the internal field c(n) and d(n) coefficients   .
c     .    the c(n) and d(n) for theta incident polarization are,  .
c     .    within an n-dependent factor, the same as the c(n) and  .
c     .    d(n) coefficients, respectively, defined in C.F.        .
c     .    Bohren and D.R. Huffman, Absorption and Scattering of   .
c     .    Light by Small Particles (Wiley- Interscience,New       .
c     .    York,1983), p.100                                       .
c     .                                                            .
c     .                             or                             .
c     .                                                            .
c     .  calculate the scattered field f(n) and g(n) coefficients  .
c     .    the f(n) and g(n) for theta incident polarization are,  .
c     .    within an n-dependent factor, the same as the b(n) and  .
c     .    a(n) coefficients, respectively, defined in C.F.        .
c     .    Bohren and D.R. Huffman, Absorption and Scattering of   .
c     .    Light by Small Particles (Wiley- Interscience,New       .
c     .    York,1983), p.100                                       .
c     .                                                            .
c     .  note: the coefficients are combined into single arrays    .
c     .    cf(*) = c(*) or f(*), dg(*) = d(*) or g(*)              .
c     ..............................................................
      complex cm,ci,z,hkl(331),bsl(227),hnjn,hnjnm,hnmjn,b,an,
     1cf(329),dg(329),amat(329)
      common /cfcom/ cf,dg
      ci = (0.0,1.0)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 4.16
c     .........................................................
c     .  the number of terms required for convergence of the  .
c     .    total external intensity at 1.5*x needs to be set  .
c     .    as if the sphere radius were 1.5*x to insure       .
c     .    convergence of the incident field expansion        .
c     .........................................................
      if(ic.eq.2) xc = (1.5*x)+4.05*(1.5*x)**.3333+2.0                  eq 4.16
      nc = int(xc)
      nci = nc+1
      z = cm*x
c     ...........................................................
c     .  calculate the Bessel functions - the order is          .
c     .    incremented by one in the hkl(*) and bsl(*) arrays  .
c     ...........................................................
      call besh(x,hkl,nci)
      if(ic.eq.1) then
c     ...............................................
c     .  calculate the internal field coefficients  .
c     ...............................................
        call besj(z,bsl,nci)
c     ................................
c     .  calculate the coefficients  .
c     ................................
        do 10 n = 1,nc
          rn = real(n)
          rf = (2.0*rn+1.0)/(rn*(rn+1.0))
          hnmjn = hkl(n)*bsl(n+1)
          hnjnm = hkl(n+1)*bsl(n)
          hnjn = hkl(n+1)*bsl(n+1)
c     .........................................................
c     .  internal field coefficients for theta                .
c     .    (parallel) incident polarization                   .
c     .    cf(n) = ci**n*rf*(Bohren and Huffman's c(n))       .
c     .    dg(n) = -ci**(n+1)*rf*(Bohren and Huffman's d(n))  .
c     .........................................................
          cf(n) = ci**n*rf*(ci/x)/((hnmjn-cm*hnjnm)*x)                  eq 4.17a
          dg(n) = -ci**(n+1)*rf*(cm*ci/x)/(x*(cm**2*hnmjn-cm*hnjnm)
     1                                        -rn*(cm**2-1.0)*hnjn)     eq 4.17b
10      continue
      else
c     ...............................................
c     .  calculate the external field coefficients  .
c     ...............................................
c     ..................................................
c     .  logarithmic derivative calculation - set the  .
c     .    starting order for downward recursion       .
c     ..................................................
        nmx = int(max(xc,abs(z)))+15                                    eq 4.20
        an = 0.0
        do 20 n = nmx,nc+1,-1
          rn = real(n)
          an = rn/z-1.0/(an+rn/z)
20      continue
        amat(nc) = an
        do 30 n = nc,2,-1
          rn = real(n)
          amat(n-1) = rn/z-1.0/(amat(n)+rn/z)                           eq 4.19
30      continue
        bj = real(hkl(1))
c     ................................
c     .  calculate the coefficients  .
c     ................................
        do 40 n = 1,nc
          rn = real(n)
          rf = 2.0*rn*(rn+1.0)
          bjm = bj
          bj = real(hkl(n+1))
c     ........................................................
c     .  scattering coefficients for theta                   .
c     .    (parallel) incident polarization                  .
c     .    cf(n) = -ci**n*rf*(Bohren and Huffman's b(n))     .
c     .    dg(n) = ci**(n+1)*rf*(Bohren and Huffman's a(n))  .
c     ........................................................
          b = cm*amat(n)+rn/x
          cf(n) = -ci**n*rf*(b*bj-bjm)/(b*hkl(n+1)-hkl(n))              eq 4.18a
          b = amat(n)/cm+rn/x
          dg(n) = ci**(n+1)*rf*(b*bj-bjm)/(b*hkl(n+1)-hkl(n))           eq 4.18b
40      continue
      end if
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
c     .  arrays are set for nc = 331 maximum            .
c     ...................................................
      complex hankel(nc)
      dimension bj(331),by(331),t(3)
c     ................................................
c     .  by(*) calculation - obtain the zeroeth and  .
c     .                      first order functions   .
c     ................................................
      a = sin(x)/x                                                      eq 4.68
      by(1) = -cos(x)/x                                                 eq 4.69a
      by(2) = by(1)/x-a                                                 eq 4.69b
c     ...........................................................
c     .  obtain the higher order functions by upward recursion  .
c     ...........................................................
        do 10 n = 3,nc
        rn = real(n-2)
        by(n) = (2.0*rn+1.0)*by(n-1)/x-by(n-2)
10      continue
c     ................................................
c     .  bj(*) calculation - set the starting order  .
c     .                      for downward recursion  .
c     ................................................
      nst = nc+int((101.0+x)**.5)                                       eq 4.21
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
20      continue
c     ...............................................
c     .  continue downward recursion to order zero  .
c     ...............................................
      bj(nc) = t(3)
      bj(nc-1) = t(2)
        do 30 i = nc-2,1,-1
        ri = real(i)
        bj(i) = (2.0*ri+1.0)*bj(i+1)/x-bj(i+2)
30      continue                
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
      alpha = a/bj(1)
        do 40 k = 1,nc
        hankel(k) = cmplx(bj(k)*alpha,by(k))
40      continue
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
      nst =int(x+4.05*x**.3333+2.0+(101.0+x)**.5)                       eq 4.21
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
10      continue
c     ...............................................
c     .  continue downward recursion to order zero  .
c     ...............................................
      bslcmp(nc) = t(3)
      bslcmp(nc-1) = t(2)
        do 20 i = nc-2,1,-1
        ri = real(i)
        bslcmp(i) = (2.0*ri+1.0)*bslcmp(i+1)/z-bslcmp(i+2)
20      continue
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
        alpha = sin(z)/(z*bslcmp(1))
        do 30 k = 1,nc
        bslcmp(k) = bslcmp(k)*alpha
30      continue
      return
      end
