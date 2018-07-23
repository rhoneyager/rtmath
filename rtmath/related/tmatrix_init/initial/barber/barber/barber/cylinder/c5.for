      program C5
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ...........................................................
c     .  calculate the intensity at the surface for a cylinder  .
c     .    illuminated at normal incidence for two cases:       .
c     .  case 1: internal intensity                             .
c     .  case 2: external intensity                             .
c     .  inputs: ic = case (internal or external)               .
c     .          ip = polarization par (TM) or perp (TE)        .
c     .          cm = complex index of refraction, (real,imag)  .
c     .               (imag is positive for absorption)         .
c     .          x = size parameter (ka)                        .
c     .          dlt = increment in angle,degrees               .
c     .                                                         .
c     .  dimension of arrays amat(*) and by(*):                 .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,    .
c     .    nc = 225                                             .
c     .                                                         .
c     .  dimension of array bj(*):                              .
c     .    nst+1 = nc+int((101.0+x)**.5)+1, e.g., for x = 200,  .
c     .    nst+1 = 243                                          .
c     .                                                         .
c     .  dimension of array aj(*) is determined by z = cm*x:    .
c     .    nst+1 = int(abs(z)+4.05*abs(z)**.3333+2.0            .
c     .                +(101.0+abs(z))**.5)+1,                  .
c     .    e.g., for x = 200, cm = 1.5, nst+1 = 350             .
c     .                                                         .
c     .  arrays are set for a maximum size parameter of 200     .
c     ...........................................................
      complex cf(225),ci,cm,cmx,aj(350),h,hp,erho,ephi,ez
      common /cfcom/ cf,aj,bj,by
      dimension bj(243),by(225)
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
      open(unit=9,file='c5.dat')
      rewind 9
      write(6,*) 'enter delta angle'
      read(5,*) dlt
      npnts = int(180./dlt)+1
      dphi = pi/real(npnts-1)
      call clndr(x,cm,ip,nc,ic)
      if(ic.eq.1) then
c     .................................................
c     .  calculate internal intensity at the surface  .
c     .................................................
        write(6,*) 'internal intensity'
        do 30 j = 1,npnts
          phi = real(j-1)*dphi
          if(ip.eq.1) then
c     .....................
c     .  TM polarization  .
c     .....................
            ez = cf(1)*aj(1)
            do 10 n = 2,nc
              rn = real(n-1)
              ez = ez+2.0*cf(n)*aj(n)*cos(rn*phi)*ci**(n-1)             eq 2.33
10          continue
            e2 = real(ez)**2+aimag(ez)**2
          else
c     .....................
c     .  TE polarization  .
c     .....................
            cmx = cm*x
            ephi = -cf(1)*(-aj(2))
            erho = 0.0
            do 20 n = 2,nc
              rn = real(n-1)
              ephi = ephi-2.0*cf(n)*(aj(n-1)-aj(n)*rn/cmx)
     1                       *cos(rn*phi)*ci**(n-1)                     eq 2.35
              erho = erho-2.0*cf(n)*rn*(aj(n)/cmx)
     1                       *sin(rn*phi)*ci**(n-1)                     eq 2.34
20          continue
            ephi = ci*ephi
            erho = ci*erho
            e2 = (real(ephi)**2+aimag(ephi)**2
     1           +real(erho)**2+aimag(erho)**2)/abs(cm)**2
          end if
          phi = rtd*phi
          write(9,110) phi,e2
30      continue
      else
c     .................................................
c     .  calculate external intensity at the surface  .
c     .................................................
        write(6,*) 'external intensity'
        do 60 j = 1,npnts
          phi = real(j-1)*dphi
          if(ip.eq.1) then
c     .....................
c     .  TM polarization  .
c     .....................
            h = cmplx(bj(1),by(1))
            ez = -cf(1)*h
            do 40 n = 2,nc
              rn = real(n-1)
              h = cmplx(bj(n),by(n))
              ez = ez-2.0*cf(n)*h*cos(rn*phi)*ci**(n-1)                 eq 2.38
40          continue
c     ......................................................
c     .  add the incident field to obtain the total field  .
c     ......................................................
            ez = ez+exp(ci*x*cos(phi))                                  eq 2.37
            e2 = real(ez)**2+aimag(ez)**2
          else
c     .....................
c     .  TE polarization  .
c     .....................
            hp = -cmplx(bj(2),by(2))
            ephi = cf(1)*hp
            erho = 0.0
            do 50 n = 2,nc
              rn = real(n-1)
              h = cmplx(bj(n),by(n))
              hp = cmplx(bj(n-1),by(n-1))-h*rn/x
              ephi = ephi+2.0*cf(n)*hp*cos(rn*phi)*ci**(n-1)            eq 2.42
              erho = erho+2.0*cf(n)*rn*h/x*sin(rn*phi)*ci**(n-1)        eq 2.41
50          continue
c     ......................................................
c     .  add the incident field to obtain the total field  .
c     ......................................................
            ephi = ci*ephi+cos(phi)*exp(ci*x*cos(phi))                  eq 2.40
            erho = ci*erho+sin(phi)*exp(ci*x*cos(phi))                  eq 2.39
            e2 = real(ephi)**2+aimag(ephi)**2
     1           +real(erho)**2+aimag(erho)**2
          end if
          phi = rtd*phi
          write(9,110) phi,e2
60      continue
      end if   
      close(unit=9)
      stop
100   format('..........................................',/,
     1       '.  calculate surface intensity vs angle  .',/,
     2       '.  output is written to c5.dat           .',/,
     3       '..........................................',//,
     4       'case: internal (1), external (2)',/,
     5       'polarization: TM (1) TE (2)',/,
     6       'size parameter: x',/,
     7       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(2e14.6)
      end
      subroutine clndr(x,cm,ip,nc,ic)
c     ........................................................
c     .  calculate the internal and scattering coefficients  .     
c     .    - the order is incremented by one                 .
c     ........................................................
      complex cf(225),amat(225),an,cm,cmm,b,z,h,hm,aj(350)
      common /cfcom/ cf,aj,bj,by
      dimension bj(243),by(225)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 2.18
      nc = int(xc)
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
c     ............................................
c     .  calculate the zeroeth mode coefficient  .
c     ............................................
      cmm = cm
      if(ip.eq.2) cmm = 1.0/cm
      h = cmplx(bj(1),by(1))
      hm = cmplx(bj(2),by(2))
      cf(1) = (cmm*amat(1)*bj(1)+bj(2))/(cmm*amat(1)*h+hm)
      if(ic.eq.1) cf(1) = (bj(1)-cf(1)*h)/aj(1)
c     ............................................
c     .  calculate the higher mode coefficients  .
c     ............................................
      do 30 n = 2,nc
        rn = real(n-1)
        b = cmm*amat(n)+rn/x
        hm = h
        h = cmplx(bj(n),by(n))
        cf(n) = (b*bj(n)-bj(n-1))/(b*h-hm)                              eq 2.19
        if(ic.eq.1) cf(n) = (bj(n)-cf(n)*h)/aj(n)                       eq 2.5
30    continue
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