      program S6
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ...........................................................
c     .  calculate the intensity at the surface of a sphere     .
c     .    for two cases:                                       .
c     .  case 1: internal intensity                             .
c     .  case 2: external intensity                             .
c     .  inputs: ic = case (internal or external)               .
c     .          ip = polarization parallel or perpendicular    .
c     .          cm = complex index of refraction, (real,imag)  .
c     .               (imag is positive for absorption)         .
c     .          x = size parameter (ka)                        .
c     .          dlt = increment in angle,degrees               .
c     .                                                         .
c     .  dimension of arrays cf(*), dg(*), amat(*),             .
c     .     and cnrm(*): nc = int(x+4.05*x**.3333+2.0),         .
c     .     e.g., for x = 200, nc = 225                         .
c     .                                                         .
c     .  dimension of arrays bj(*), by(*), hkl(*), bsl(*),      .
c     .    and pnmllg(*): nc+1, e.g., for x = 200, nc+1 = 226   .
c     .                                                         .
c     .  arrays are set for a maximum size parameter of 200     .
c     ...........................................................
      complex cm,ci,hkl(226),bsl(226),b,er,eth,eph,cmx,amag,
     1cf(225),dg(225)
      common /cfcom/ cf,dg,cnrm,hkl,bsl
      dimension pnmllg(226),cnrm(225)
      pi = 3.14159265358979
      rtd = 180.0/pi
      ci = (0.0,1.0)
      write(6,100)
      write(6,*) 'enter case, polarization, size parameter, mr, mi'
      read(5,*) ic,ip,x,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='s6.dat')
      rewind 9
      write(6,*) 'enter delta angle'
      read(5,*) dlt
      nang = int(180./dlt)+1
      dlt = pi/real(nang-1)
      call sphere(x,cm,ip,nc,ic)
      nci = nc+1
      if(ic.eq.1) then
c     .................................................
c     .  calculate internal intensity at the surface  .
c     .................................................
        write(6,*) 'internal intensity'
        cmx = cm*x
        do 20 i = 1,nang
          theta = real(i-1)*dlt 
          sinth = sin(theta)
          costh = cos(theta)
          call genlgp(theta,pnmllg,nci)
          er = 0.0
          eth = 0.0
          eph = 0.0
          do 10 n = 1,nc
            n1 = n+1
            rn = real(n)
            p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
            p2 = pnmllg(n1)
            b = bsl(n)-rn*bsl(n1)/cmx
c     ..................................
c     .  calculate the electric field  .
c     .    in the phi = 0 plane        .
c     .    sin(phi) = sin(0) = 0.0     .
c     .    cos(phi) = cos(0) = 1.0     .
c     ..................................
            if(ip.eq.1) then
c     ..................................................
c     .  parallel (theta) incident polarization gives  .
c     .    radial and theta internal polarization      .
c     ..................................................
              er = er+rn*(rn+1.0)*(bsl(n1)/cmx)*p2*sinth*dg(n)          eq 4.40a
              eth = eth+bsl(n1)*p2*cf(n)+b*p1*dg(n)                     eq 4.40b
            else
c     .........................................................
c     .  perpendicular (phi) incident polarization gives phi  .
c     .    internal polarization.                             .
c     .........................................................
              eph = eph-bsl(n1)*p1*cf(n)+b*p2*dg(n)                     eq 4.41
            end if
10        continue
          e2 = abs(er)**2+abs(eth)**2+abs(eph)**2                       eq 4.35
          theta = rtd*theta
          write(9,110) theta,e2
20      continue
      else
c     .................................................
c     .  calculate external intensity at the surface  .
c     .................................................
        write(6,*) 'external intensity'
        do 40 i = 1,nang
          theta = real(i-1)*dlt
          sinth = sin(theta)
          costh = cos(theta)
          call genlgp(theta,pnmllg,nci)
          er = 0.0
          eth = 0.0
          eph = 0.0
          do 30 n = 1,nc
            n1 = n+1
            rn = real(n)
            p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
            p2 = pnmllg(n1)
            b = hkl(n)-rn*hkl(n1)/x
c     ..................................
c     .  calculate the electric field  .
c     .    in the phi = 0 plane        .
c     .    sin(phi) = sin(0) = 0.0     .
c     .    cos(phi) = cos(0) = 1.0     .
c     ..................................
            if(ip.eq.1) then
c     ..................................................
c     .  parallel (theta) incident polarization gives  .
c     .    radial and theta scattered polarization     .
c     ..................................................
              er = er+rn*(rn+1.0)*(hkl(n1)/x)*p2*sinth*dg(n)
     1                                                 *cnrm(n)         eq 4.44a
              eth = eth+(hkl(n1)*p2*cf(n)+b*p1*dg(n))*cnrm(n)           eq 4.44b
            else
c     .........................................................
c     .  perpendicular (phi) incident polarization gives phi  .
c     .    scattered polarization.                            .
c     .........................................................
              eph = eph-(hkl(n1)*p1*cf(n)-b*p2*dg(n))*cnrm(n)           eq 4.45
            end if
30        continue
c     ......................................................
c     .  add the incident field to obtain the total field  .
c     ......................................................
          amag = exp(ci*x*costh)
          if(ip.eq.1) then                                              add
            er = er+amag*sinth                                          eq 4.42
            eth = eth+amag*costh                                        eq 4.42
            e2 = abs(er)**2+abs(eth)**2                                 eq 4.35
          else                                                          add
            eph = eph+amag                                              eq 4.43
            e2 = abs(eph)**2                                            eq 4.35
          end if
          theta = rtd*theta
          write(9,110) theta,e2
40      continue
      end if   
      close(unit=9)
      stop
100   format('..........................................',/,
     1       '.  calculate surface intensity vs angle  .',/,
     2       '.  output is written to s6.dat           .',/,
     3       '..........................................',//,
     4       'case: internal (1), external (2)',/,
     5       'polarization: parallel (1) perpendicular (2)',/,
     6       'size parameter: x',/,
     7       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(2e14.6)
      end
      subroutine sphere(x,cm,ip,nc,ic)
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
      complex cm,ci,z,hkl(226),bsl(226),hnjn,hnjnm,hnmjn,b,an,
     1cf(225),dg(225),amat(225)
      common /cfcom/ cf,dg,cnrm,hkl,bsl
      dimension cnrm(225)
      ci = (0.0,1.0)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 4.16
      nc = int(xc)
      nci = nc+1
      z = cm*x
c     ..........................................................
c     .  calculate the Bessel functions - the order is         .
c     .    incremented by one in the hkl(*) and bsl(*) arrays  .
c     ..........................................................
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
          if(ip.eq.2) then
c     .............................................
c     .  internal field coefficients for phi      .
c     .    (perpendicular) incident polarization  .
c     .............................................
            cf(n) = -cf(n)                                              eq 4.5a
            dg(n) = dg(n)                                               eq 4.5b
          end if
10      continue
      else
c     ................................................
c     .  calculate the scattered field coefficients  .
c     ................................................
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
          if(ip.eq.2) then
c     .............................................
c     .  scattering coefficients for phi          .
c     .    (perpendicular) incident polarization  .
c     .............................................
            cf(n) = -cf(n)                                              eq 4.8a
            dg(n) = dg(n)                                               eq 4.8b
          end if
c     ........................................
c     .  calculate the normalization factor  .
c     .    (used in main program)            .
c     ........................................
        cnrm(n) = (2.0*rn+1.0)/(rf*rn*(rn+1.0))
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