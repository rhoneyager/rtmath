      program C3
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ...........................................................
c     .  calculate a particular coefficient vs size parameter   .
c     .    for an infinite cylinder illuminated at normal       .
c     .    incidence                                            .
c     .  inputs: ic = case (internal or scattered coefficient)  .
c     .          md = mode number n                             .
c     .          ip = polarization par (TM) or perp (TE)        .
c     .          cm = complex index of refraction, (real,imag)  .
c     .               (imag is positive for absorption)         .
c     .          x = size parameter (ka)                        .
c     .              x1 = lowest value, xn = highest value      .
c     .          npnts = number of points                       .
c     .                                                         .
c     .  dimension of arrays amat(*) and by(*):                 .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,    .
c     .    nc = 225                                             .
c     .                                                         .
c     .  dimension of array bj(*):                              .
c     .    nst+1 = nc+int((101.0+x)**.5)+1, e.g., for x = 200   .
c     .    nst+1 = 243                                          .
c     .                                                         .
c     .  dimension of array aj(*) is determined by z = cm*x:    .
c     .    nst+1 = int(abs(z)+4.05*abs(z)**.3333+2.0            .
c     .                +(101.0+abs(z))**.5)+1,                  .
c     .    e.g., for x = 200, cm = 1.5, nst+1 = 350             .
c     .                                                         .
c     .  arrays are set for a maximum size parameter of 200     .
c     .  note: maximum allowable value of md is 224 with        .
c     .          array dimensions set as above                  .
c     ...........................................................
      complex cf,cm
      common /cfcom/ cf
      write(6,100)
      write(6,*) 'enter case, mode number, polarization, mr, mi'
      read(5,*) ic,md,ip,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      write(6,*) 'enter x1, xn, npnts'
      read(5,*) x1,xn,npnts
      write(6,*) 'enter delta in x (1) or inverse x (2)'
      read(5,*) id
      open(unit=9,file='c3.dat')
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
c     .................................................................
c     .  calculate real and imaginary coefficients vs size parameter  .
c     .................................................................
      do 10 i = 1,npnts
        call clndr(x,cm,ip,md,ic)
        ri = real(i)
        if(id.eq.2) then
          xi = 1.0/x
          write(9,110) xi,cf
          x = 1.0/(ri*dwl+xni)
        else
          write(9,110) x,cf
          x = x1+ri*dk
        end if
10    continue
      close(unit=9)
      stop
100   format('..............................................',/,
     1       '.  calculate coefficients vs size parameter  .',/,
     2       '.  output is written to c3.dat               .',/,
     3       '..............................................',//,
     4       'case: internal (1) or scattered (2) coefficient',/,
     5       'mode: mode number n',/,
     6       'polarization: TM (1) TE (2)',/,
     7       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(3e14.6)
      end
      subroutine clndr(x,cm,ip,md,ic)
c     ........................................................
c     .  calculate the internal and scattering coefficients  .
c     .    - the order is incremented by one                 .     
c     ........................................................
      complex cf,amat(225),an,cm,cmm,b,z,h,hm,aj(350)
      common /cfcom/ cf
      dimension bj(243),by(225)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 2.18
      nc = int(xc)
      if(md.ge.nc) nc = md+1
      z = cm*x
c     .......................................................
c     .  logarithmic derivative calculation - the order is  .
c     .    incremented by one in the amat(*) array          .
c     .  set the starting order for downward recursion      .
c     .......................................................
      nmx = int(max(xc,abs(z)))+15                                      eq 2.22
      an = 0.0
      do 10 n = nmx,nc,-1
        rn = real(n)
        an = (rn-1.0)/z-1.0/(an+rn/z)
10    continue
      amat(nc) = an
      do 20 n = nc-1,2,-1
        rn = real(n)
        amat(n) = (rn-1.0)/z-1.0/(amat(n+1)+rn/z)                       eq 2.21
20    continue
      amat(1) = -1.0/(amat(2)+1.0/z)
c     ...............................................................
c     .  calculate the Bessel functions - the order is              .
c     .    incremented by one in the bj(*), by(*) and aj(*) arrays  .
c     ...............................................................
      call besh(x,bj,by,nc)
      if(ic.eq.1) call besj(z,aj,nc)
c     .......................................................
c     .  calculate the zeroeth mode scattering coefficient  .
c     .......................................................
      cmm = cm
      if(ip.eq.2) cmm = 1.0/cm
      if(md.eq.0) then
        h = cmplx(bj(1),by(1))
        hm = cmplx(bj(2),by(2))
        cf = (cmm*amat(1)*bj(1)+bj(2))/(cmm*amat(1)*h+hm)
        if(ic.eq.1) cf = (bj(1)-cf*h)/aj(1)
      else 
c     .......................................................
c     .  calculate the higher mode scattering coefficients  .
c     .......................................................
        n = md+1
        rn = real(n-1)
        b = cmm*amat(n)+rn/x
        hm = cmplx(bj(n-1),by(n-1))
        h = cmplx(bj(n),by(n))
        cf = (b*bj(n)-bj(n-1))/(b*h-hm)                                 eq 2.19
        if(ic.eq.1) cf = (bj(n)-cf*h)/aj(n)                             eq 2.5
      end if
      return
      end
      subroutine besh(x,bj,by,nc)
c     ......................................................
c     .  calculate Bessel functions                        .
c     .  bj = Bessel function of the first kind            .
c     .  by = Bessel function of the second kind           .
c     .  x = real argument                                 . 
c     .  nc = number of orders (0 to nc-1)                 .
c     .  the order of the Bessel functions is incremented  .
c     .    by one in the bj(*) and by(*) arrays            .
c     ......................................................
      dimension bj(243),by(nc)
      gamma = .577215664901533
      pi = 3.14159265358979
c     ................................................
c     .  bj(*) calculation - set the starting order  .
c     .                      for downward recursion  .
c     ................................................
      nst = nc+int((101.0+x)**.5)                                       eq 2.23
c     ....................................................
c     .  set starting values for the two highest orders  .
c     ....................................................
      bj(nst+1) = 0.0
      bj(nst) = 1.0e-35
c     ...................................................
c     .  obtain scaled functions by downward recursion  .
c     ...................................................
      do 10 i = nst-1,1,-1
        ri = real(i)
        bj(i) = 2.0*ri*bj(i+1)/x-bj(i+2)
10    continue
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
      alpha = bj(1)
      do 20 j = 2,nst-2,2
        alpha = alpha+2.0*bj(j+1)                                       eq 2.53
20    continue
      do 30 k = 1,nst-1
        bj(k) = bj(k)/alpha
30    continue
c     ................................................
c     .  by(*) calculation - obtain the zeroeth and  .
c     .                      first order functions   .
c     ................................................
      by(1) = bj(1)*(log(x/2.0)+gamma)
      do 40 m = 1,nst/2-1
        rm = real(m)
        by(1) = by(1)-2.0*((-1.0)**m)*bj(2*m+1)/rm
40    continue
      by(1) = 2.0*by(1)/pi
      by(2) = (bj(2)*by(1)-2.0/(pi*x))/bj(1)
c     ...........................................................
c     .  obtain the higher order functions by upward recursion  .
c     ...........................................................
      do 50 n = 3,nc
        rn = real(n-2)
        by(n) = 2.0*rn*by(n-1)/x-by(n-2)
50    continue
      return
      end
      subroutine besj(z,aj,nc)
c     ......................................................
c     .  calculate Bessel functions of the first kind      .
c     .  aj = Bessel function of the first kind            .
c     .  x = complex argument                              . 
c     .  nc = number of orders (0 to nc-1)                 .
c     .  the order of the Bessel functions is incremented  .
c     .    by one in the aj array                          .
c     ......................................................
      complex aj(350),z,alpha
c     ...................................................
c     .  set the starting order for downward recursion  .
c     ...................................................
      x = abs(z)
      nst =int(x+4.05*x**.3333+2.0+(101.0+x)**.5)                       eq 2.23
c     ....................................................
c     .  set starting values for the two highest orders  .
c     ....................................................
      aj(nst+1) = 0.0
      aj(nst) = 1.0e-35
c     ...................................................
c     .  obtain scaled functions by downward recursion  .
c     ...................................................
      do 10 i = nst-1,1,-1
        ri = real(i)
        aj(i) = 2.0*ri*aj(i+1)/z-aj(i+2)
10    continue
c     ..................................................
c     .  calculate the scale factor and the functions  .
c     ..................................................
      alpha = aj(1)
      do 20 j = 2,nst-2,2
        alpha = alpha+2.0*aj(j+1)                                       eq 2.53
20    continue
      do 30 k = 1,nc
        aj(k) = aj(k)/alpha
30    continue
      return
      end
