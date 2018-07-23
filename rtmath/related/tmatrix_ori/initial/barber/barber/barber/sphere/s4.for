      program S4
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ...........................................................
c     .  calculate a particular expansion coefficient vs size   .
c     .    parameter for a sphere                               .
c     .                                                         .
c     .  calculate the internal field c(n) and d(n) or the      .
c     .    scattered field a(n) and b(n) coefficients - both    .
c     .    sets as defined in C.F. Bohren and D.R. Huffman      .
c     .    Absorption and Scattering of Light by Small Part-    .
c     .    icles (Wiley-Interscience,New York,1983), p.100      .
c     .                                                         .
c     .  inputs: ic = case (internal or scattered coefficient)  .
c     .          md = mode number n                             .
c     .          cm = complex index of refraction, (real,imag)  .
c     .               (imag is positive for absorption)         .
c     .          x = size parameter (ka)                        .
c     .              x1 = lowest value, xn = highest value      .
c     .          npnts = number of points                       .
c     .                                                         .
c     .  dimension of array amat(*):                            .
c     .     nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,   .
c     .     nc = 225                                            .
c     .                                                         .
c     .  dimension of arrays bj(*), by(*), hkl(*), bsl(*):      .
c     .    nc+1, e.g., for x = 200, nc+1 = 226                  .
c     .                                                         .
c     .  arrays are set for a maximum size parameter of 200     .
c     .  note: maximum allowable value of md is 225 with        .
c     .          array dimensions set as above                  .
c     ...........................................................
      complex ca,db,cm
      common /cfcom/ ca,db
      write(6,100)
      write(6,*) 'enter case, mode number, mr, mi'
      read(5,*) ic,md,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      write(6,*) 'enter x1, xn, npnts'
      read(5,*) x1,xn,npnts
      write(6,*) 'enter delta in x (1) or wavelength (2)'
      read(5,*) id
      open(unit=9,file='s4.dat')
      rewind 9
      if(id.eq.2) then
        x1i = 1.0/x1
        xni = 1.0/xn
        dwl = 0.0
        if(npnts.ne.1) dwl = (x1i-xni)/real(npnts-1)
        x = xn
      else
        dk = 0.0
        if(npnts.ne.1) dk = (xn-x1)/real(npnts-1)
        x = x1
      end if
c     ..............................................
c     .  calculate coefficients vs size parameter  .
c     ..............................................
        do 10 i = 1,npnts
        call sphere(x,cm,md,ic)
        ri = real(i)
        if(id.eq.2) then
          xi = 1.0/x
          write(9,110) xi,ca,db
          x = 1.0/(ri*dwl+xni)
        else
          write(9,110) x,ca,db
          x = x1+ri*dk
        end if
10      continue
      close(unit=9)
      stop
100   format('..............................................',/,
     1       '.  calculate coefficients vs size parameter  .',/,
     2       '.  c(n) and d(n) or a(n) and b(n)            .',/,
     3       '.  output is written to s4.dat               .',/,
     4       '..............................................',//,
     5       'case: internal (1) or scattered (2) coefficient',/,
     6       'mode: mode number n',/,
     7       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(5e14.6)
      end
      subroutine sphere(x,cm,md,ic)
c     ...............................................................
c     .  calculate the internal field c(n) and d(n) coefficients    .
c     .                             or                              .
c     .  calculate the scattered field a(n) and b(n) coefficients   .
c     .    both sets as defined in C.F. Bohren and D.R. Huffman,    .
c     .    Absorption and Scattering of Light by Small Particles    .
c     .    (Wiley-Interscience,New York,1983), p.100                .
c     .                                                             .
c     .  note: the coefficients are combined into single variables  .
c     .    ca = c or a,  db = d or b                                .
c     ...............................................................
      complex cm,ci,z,hkl(226),bsl(226),hnjn,hnjnm,hnmjn,b,an,
     1ca,db,amat(225)
      common /cfcom/ ca,db
      ci = (0.0,1.0)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 4.16
      nc = int(xc)
      if(md.gt.nc) nc = md
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
        n = md
        rn = real(n)
        hnmjn = hkl(n)*bsl(n+1)
        hnjnm = hkl(n+1)*bsl(n)
        hnjn = hkl(n+1)*bsl(n+1)
        ca = (ci/x)/((hnmjn-cm*hnjnm)*x)                                eq 4.17a
        db = (cm*ci/x)/(x*(cm**2*hnmjn-cm*hnjnm)-rn*(cm**2-1.0)*hnjn)   eq 4.17b
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
c     ................................
c     .  calculate the coefficients  .
c     ................................
        n = md
        rn = real(n)
        bjm = real(hkl(n))
        bj = real(hkl(n+1))
        b = amat(n)/cm+rn/x
        ca = (b*bj-bjm)/(b*hkl(n+1)-hkl(n))                             eq 4.18a
        b = cm*amat(n)+rn/x
        db = (b*bj-bjm)/(b*hkl(n+1)-hkl(n))                             eq 4.18b
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
