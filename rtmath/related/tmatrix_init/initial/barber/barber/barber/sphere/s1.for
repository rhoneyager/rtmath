      program S1
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .............................................................
c     .  calculate the scattering by a sphere for three cases:    .
c     .  case 1: efficiency (Qe, Qs and Qa) vs size parameter     .
c     .  case 2: intensity at a specific angle vs size parameter  . 
c     .  case 3: intensity vs scattering angle                    .
c     .  inputs: ic = case                                        .
c     .          ip = polarization parallel or perpendicular      .
c     .          cm = complex index of refraction, (real,imag)    .
c     .               (imag is positive for absorption)           .
c     .          x = size parameter (ka)                          .
c     .              x1 = lowest value, xn = highest value        .
c     .          angle = scattering angle, degrees                .
c     .          dlt = increment in scattering angle, degrees     .
c     .          npnts = number of points                         .
c     .                                                           .
c     .  dimension of arrays f(*), g(*), amat(*) and cnrm(*):     .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,      .
c     .    nc = 225                                               .
c     .                                                           .
c     .  dimension of arrays bj(*), by(*), hkl(*), and            .
c     .    pnmllg(*): nc+1, e.g., for x = 200, nc+1 = 226         .
c     .                                                           .
c     .  arrays are set for a maximum size parameter of 200       .
c     .............................................................
      complex cm,ci,cim,f(225),g(225),t
      common /cfcom/ f,g,cnrm
      dimension pnmllg(226),cnrm(225)
      pi = 3.14159265358979
      rtd = 180.0/pi
      ci = (0.0,1.0)
      write(6,100)
      write(6,*) 'enter case, polarization, mr, mi'
      read(5,*) ic,ip,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='s1.dat')
      rewind 9
      if(ic.eq.1) then
c     ............................................
c     .  calculate efficiency vs size parameter  .
c     ............................................
        write(6,*) 'Qe, Qs, and Qa vs x - - - enter x1, xn, npnts'
        read(5,*) x1,xn,npnts
        write(6,*) 'enter delta in x (1) or inverse x (2)'
        read(5,*) id
        if(id.eq.1) then
c     .....................................................
c     .  set increments for efficiency vs size parameter  .
c     .....................................................
          dk = 0.0
          if(npnts.ne.1) dk = (xn-x1)/real(npnts-1)
          x = x1
        else
c     .............................................................
c     .  set increments for efficiency vs inverse size parameter  .
c     .............................................................
          x1i = 1.0/x1
          xni = 1.0/xn
          dwl = 0.0
          if(npnts.ne.1) dwl = (x1i-xni)/real(npnts-1)
          x = xn
        end if
        do 30 i = 1,npnts
          ip = 1
          call sphere(x,cm,ip,nc)
c     .........................................
c     .  calculate the extinction efficiency  .
c     .........................................
          qext = 0.0
          do 10 n = 1,nc
            n1 = n+1
            cim = ci**(-n1)
            rn = real(n)
            rf = (2.0*rn+1.0)/(rn*(rn+1.0))
            qext = qext+rf*aimag(cim*(f(n)+ci*g(n)))                    eq 4.13
10        continue            
          qext = qext/x**2
c     ..........................................
c     .   calculate the scattering efficiency  .
c     ..........................................
          qsca = 0.0
          do 20 n = 1,nc
            qsca = qsca+(abs(f(n))**2+abs(g(n))**2)*cnrm(n)             eq 4.14
20        continue
          qsca = qsca/x**2
c     ..........................................
c     .   calculate the absorption efficiency  .
c     ..........................................
          qabs = qext-qsca                                              eq 4.15
          ri = real(i)
          if(id.eq.1) then
            write(9,110) x,qext,qsca,qabs
            x = x1+ri*dk
          else
            xi = 1.0/x
            write(9,110) xi,qext,qsca,qabs
            x = 1.0/(ri*dwl+xni)
          end if
30      continue
       else if(ic.eq.2) then          
c     ...........................................
c     .  calculate intensity vs size parameter  .
c     ...........................................
        write(6,*) 'I vs x - - - enter x1, xn, npnts, angle'
        read(5,*) x1,xn,npnts,theta
        theta = theta/rtd
        costh = cos(theta)
        write(6,*) 'enter delta in x (1) or inverse x (2)'
        read(5,*) id
        if(id.eq.1) then
c     ....................................................
c     .  set increments for intensity vs size parameter  .
c     ....................................................
          dk = 0.0
          if(npnts.ne.1) dk = (xn-x1)/real(npnts-1)
          x = x1
        else
c     ............................................................
c     .  set increments for intensity vs inverse size parameter  .
c     ............................................................
          x1i = 1.0/x1
          xni = 1.0/xn
          dwl = 0.0
          if(npnts.ne.1) dwl = (x1i-xni)/real(npnts-1)
          x = xn
        end if
        do 50 i = 1,npnts
          snorm = 1.0/(pi*x**2)
          call sphere(x,cm,ip,nc)
          nci = nc+1
          call genlgp(theta,pnmllg,nci)
          t = 0.0
          do 40 n = 1,nc
            n1 = n+1
            cim = ci**(-n1)
            rn = real(n)
            p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
            p2 = pnmllg(n1)
            if(ip.eq.1) then
c     ..........................................
c     .  calculate parallel polarization case  .
c     ..........................................
              t = t+cim*(p2*f(n)+ci*p1*g(n))*cnrm(n)                    eq 4.22
            else
c     ...............................................
c     .  calculate perpendicular polarization case  .
c     ...............................................
              t = t+cim*(-p1*f(n)+ci*p2*g(n))*cnrm(n)                   eq 4.23
            end if  
40        continue
          tp = snorm*abs(t)**2
          ri = real(i)
          if(id.eq.1) then
            write(9,120) x,tp
            x = x1+ri*dk
          else
            xi = 1.0/x
            write(9,120) xi,tp
            x = 1.0/(ri*dwl+xni)
          end if
50      continue
      else
c     .............................................
c     .  calculate intensity vs scattering angle  .
c     .............................................
        write(6,*) 'I vs angle - - - enter x, delta angle'
        read(5,*) x,dlt
        snorm = 1.0/(pi*x**2)
        nang = int(180.0/dlt)+1
        dlt = pi/real(nang-1)
        call sphere(x,cm,ip,nc)
        nci = nc+1
        do 70 i = 1,nang
          theta = real(i-1)*dlt
          costh = cos(theta)
          call genlgp(theta,pnmllg,nci)
          t = 0.0
          do 60 n = 1,nc
            n1 = n+1
            cim = ci**(-n1)
            rn = real(n)
            p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
            p2 = pnmllg(n1)
            if(ip.eq.1) then
c     ..........................................
c     .  calculate parallel polarization case  .
c     ..........................................
              t = t+cim*(p2*f(n)+ci*p1*g(n))*cnrm(n)                    eq 4.22
            else
c     ...............................................
c     .  calculate perpendicular polarization case  .
c     ...............................................
              t = t+cim*(-p1*f(n)+ci*p2*g(n))*cnrm(n)                   eq 4.23
            end if  
60        continue
          tp = snorm*abs(t)**2
          theta = theta*rtd
          write(9,120) theta,tp
70      continue
      end if 
      close(unit=9)
      stop
100   format('...................................................',/,
     1       '.  calculate efficiencies or scattered intensity  .',/,
     2       '.  output is written to s1.dat                    .',/,
     3       '...................................................',//,
     4       'case: Q vs x (1), I vs x (2), I vs angle (3)',/,
     5       'polarization: parallel (1) perpendicular (2)',/,
     6       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(4e14.6)
120   format(2e14.6)
      end
      subroutine sphere(x,cm,ip,nc)
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
        if(ip.eq.2) then
c     ...............................................
c     .  scattering coefficients for phi            .
c     .    (perpendicular) incident polarization    .
c     ...............................................
          f(n) = -f(n)                                                  eq 4.8a
          g(n) = g(n)                                                   eq 4.8b
        end if
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