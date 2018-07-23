      program S3
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ...........................................................
c     .  calculate the elements of the scattering matrix        .
c     .    for a sphere                                         .
c     .  inputs: x = size parameter (ka)                        .
c     .          cm = complex index of refraction, (real,imag)  .
c     .               (imag is positive for absorption)         .
c     .          dlt = increment in scattering angle, degrees   .
c     .                                                         .
c     .  dimension of arrays f(*), g(*), amat(*) and cnrm(*):   .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,    .
c     .    nc = 225                                             .
c     .                                                         .
c     .  dimension of arrays bj(*), by(*), hkl(*), and          .
c     .    pnmllg(*): nc+1, e.g., for x = 200, nc+1 = 226       .
c     .                                                         .
c     .  arrays are set for a maximum size parameter of 200     .
c     ...........................................................
      complex cm,ci,cim,f(225),g(225),s1,s2
      common /cfcom/ f,g,cnrm
      dimension pnmllg(226),cnrm(225)
      pi = 3.14159265358979
      rtd = 180.0/pi
      ci = (0.0,1.0)
      write(6,100)
      write(6,*) 'enter x, mr, mi, delta angle'
      read(5,*) x,cmr,cmi,dlt
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='s3.dat')
      rewind 9
      nang = int(180.0/dlt)+1
      dlt = pi/real(nang-1)
      call sphere(x,cm,nc)
      nci = nc+1
      t = 0.0
c     .............................................................
c     .  calculate t = Qsca*x**2                                  .
c     .              = (scattering cross section)*x**2/(pi*a**2)  .
c     .............................................................
      do 10 n = 1,nc
        t = t+(abs(f(n))**2+abs(g(n))**2)*cnrm(n)
10    continue
      do 30 i = 1,nang
        theta = real(i-1)*dlt
        costh = cos(theta)
        call genlgp(theta,pnmllg,nci)
        s1 = 0.0
        s2 = 0.0
        do 20 n = 1,nc
          n1 = n+1
          cim = ci**(-n1)
          rn = real(n)
          p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
          p2 = pnmllg(n1)
c     ........................................
c     .  calculate parallel field amplitude  .
c     .    for parallel incident             .                          eq 3.50b
c     ........................................                          and
          s2 = s2+cim*(-ci*p2*f(n)+p1*g(n))*cnrm(n)                     eq 4.10a
c     .............................................
c     .  calculate perpendicular field amplitude  .
c     .    for perpendicular incident             .
c     .  use coefficients for parallel incident   .
c     .    f(e1n) = -f(o1n),  g(o1n) = g(ein)     .                     eq 3.50a
c     .............................................                     and
          s1 = s1+cim*(ci*p1*(-f(n))+p2*g(n))*cnrm(n)                   eq 4.11b
20        continue
          theta = theta*rtd
c     ....................................................
c     .  calculate P11, PL = -P12/P11, P33/P11, P34/P11  .              eq 4.25
c     ....................................................              and
          p11 = 2.0*(abs(s1)**2+abs(s2)**2)/t                           eq 4.26a
          pl = -2.0*(abs(s2)**2-abs(s1)**2)/(t*p11)                     eq 4.26b
          p33p11 = 4.0*real(s1*conjg(s2))/(t*p11)                       eq 4.26c
          p34p11 = 4.0*aimag(s2*conjg(s1))/(t*p11)                      eq 4.26d
          write(9,110) theta,p11,pl,p33p11,p34p11
30      continue
        close(unit=9)
        stop
100   format('..........................................',/,
     1       '.  calculate scattering matrix elements  .',/,
     2       '.  P11, -P12/P11, P33/P11, and P34/P11   .',/,
     3       '.  output is written to s3.dat           .',/,
     4       '..........................................',//,
     5       'size parameter: x',/,
     6       'index of refraction: real,imaginary (+ for absorption)',/,
     7       'delta angle: degrees',/)
110   format(5e14.6)
      end
      subroutine sphere(x,cm,nc)
c     ..............................................................
c     .  calculate the scattered field f(n) and g(n) coefficients  .
c     .    the f(n) and g(n) for theta incident polarization are,  .
c     .    within an n-dependent factor, the same as the b(n) and  .
c     .    a(n) coefficients, respectively, defined in C.F.        .
c     .    Bohren and D.R. Huffman, Absorption and Scattering of   .
c     .    Light by Small Particles (Wiley- Interscience,New       .
c     .    York,1983), p.100                                       .
c     ..............................................................
      complex b,z,cm,ci,hkl(226),an,amat(225),f(225),g(225)
      common /cfcom/ f,g,cnrm
      dimension cnrm(225)
      ci = (0.0,1.0)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 4.16
      nc = int(xc)
      nci = nc+1
      z = cm*x
c     ..................................................
c     .  logarithmic derivative calculation - set the  .
c     .    starting order for downward recursion       .
c     ..................................................
      nmx = int(max(xc,abs(z)))+15                                      eq 4.20
      an = 0.0
      do 10 n = nmx,nc+1,-1
        rn = real(n)
        an = rn/z-1.0/(an+rn/z)
10    continue
      amat(nc) = an
      do 20 n = nc,2,-1
        rn = real(n)
        amat(n-1) = rn/z-1.0/(amat(n)+rn/z)                             eq 4.19
20    continue
c     ...................................................
c     .  calculate the Bessel functions - the order is  .
c     .    incremented by one in the hkl(*) array       .
c     ...................................................
      call besh(x,hkl,nci)
      bj = real(hkl(1))
c     ................................
c     .  calculate the coefficients  .
c     ................................
      do 30 n = 1,nc
        rn = real(n)
        rf = 2.0*rn*(rn+1.0)
        bjm = bj
        bj = real(hkl(n+1))
c     .......................................................
c     .  scattering coefficients for theta                  .
c     .    (parallel) incident polarization                 .
c     .    f(n) = -ci**n*rf*(Bohren and Huffman's b(n))     .
c     .    g(n) = ci**(n+1)*rf*(Bohren and Huffman's a(n))  .
c     .......................................................
        b = cm*amat(n)+rn/x
        f(n) = -ci**n*rf*(b*bj-bjm)/(b*hkl(n+1)-hkl(n))                 eq 4.18a
        b = amat(n)/cm+rn/x
        g(n) = ci**(n+1)*rf*(b*bj-bjm)/(b*hkl(n+1)-hkl(n))              eq 4.18b
c     ........................................
c     .  calculate the normalization factor  .
c     .    (used in main program)            .
c     ........................................
        cnrm(n) = (2.0*rn+1.0)/(rf*rn*(rn+1.0))
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
c     .  arrays are set for nc = 226 maximum            .
c     ...................................................
      complex hankel(nc)
      dimension bj(226),by(226),t(3)
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
      subroutine genlgp(theta,pnmllg,nc)
c     ........................................................
c     .  calculate associated Legendre functions (argument   .
c     .    cos(theta)) divided by sin(theta) for m = 1       .
c     .  generate first two orders by formula and remaining  .
c     .    orders by recursion                               .
c     .                                                      .
c     .  pnmllg = associated Legendre function/sin(theta)    .
c     .  nc = number of orders (0 to nc-1)                   .
c     .  the order of the associated Legendre functions is   .
c     .    incremented by one in the pnmllg(*) array         .
c     ........................................................
      dimension pnmllg(nc)
      costh = cos(theta)
c     ..............................
c     .  calculate orders 0 and 1  .
c     ..............................
      pnmllg(1) = 0.0                                                   eq 4.70a
      pnmllg(2) = 1.0                                                   eq 4.70b
c     .................................................
c     .  recur upward to obtain all remaining orders  .
c     .................................................
      do 10 n = 3,nc 
      rn = real(n-1)
      pnmllg(n) = ((2.0*rn-1.0)*costh*pnmllg(n-1)
     1             -rn*pnmllg(n-2))/(rn-1.0)                            eq 4.71
10    continue 
      return 
      end 
