      program C4
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ............................................................
c     .  calculate the intensity (integrated over 360 degrees)   .
c     .    at a particular radius for a cylinder illuminated at  .
c     .    normal incidence for two cases:                       .
c     .  case 1: r/a .le. 1           (internal)                 .
c     .  case 2: 1 .ge. r/a .le. 1.5  (external)                 .
c     .  inputs: ic = case (internal or external)                .
c     .          ip = polarization par (TM) or perp (TE)         .
c     .          x = size parameter (ka)                         .
c     .          cm = complex index of refraction, (real,imag)   .
c     .               (imag is positive for absorption)          .
c     .          npnts = number of points                        .
c     .                                                          .
c     .  dimension of arrays cf(*), cfa(*), amat(*) and by(*):   .
c     .    nc = int((1.5*x)+4.05*(1.5*x)**.3333+2.0),            .
c     .      e.g., for x = 200, nc = 329                         .
c     .                                                          .
c     .  dimension of array bj(*):                               .
c     .    nst+1 = nc+int((101.0+(1.5*x))**.5)+1,                .
c     .      e.g., for x = 200, nst+1 = 350                      .
c     .                                                          .
c     .  dimension of array aj(*) is determined by z = cm*x:     .
c     .    nst+1 = int(abs(z)+4.05*abs(z)**.3333+2.0             .
c     .                +(101.0+abs(z))**.5)+1,                   .
c     .    e.g., for x = 200, cm = 1.5, nst+1 = 350              .
c     .                                                          .
c     .  arrays are set for a maximum size parameter of 200      .
c     ............................................................
      complex cf(329),cm,aj(350),dx,cmx,h,hp
      common /cfcom/ cf
      dimension cfa(329),bj(350),by(329)
      write(6,100)
      write(6,*) 'enter case, polarization, size parameter, mr, mi'
      read(5,*) ic,ip,x,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='c4.dat')
      rewind 9
      if(ic.eq.1) then
c     .............................................
c     .  calculate internal integrated intensity  .
c     .............................................
        write(6,*) 'internal intensity - - - enter npnts ( >1 )'
        read(5,*) npnts
        cpm = real(npnts-1)
        dx = cm*x/cpm
        call clndr(x,cm,ip,nc,ic)
        do 10 i = 1,nc
          cfa(i) = real(cf(i))**2+aimag(cf(i))**2
10      continue
        rj = 0.0
        cm2 = abs(cm)**2
        e2 = cfa(1)                                                     eq 2.28
        if(ip.eq.2) e2 = cfa(2)/cm2                                     eq 2.30
        write(9,110) rj,e2
        do 40 j = 2,npnts
          rj = real(j-1)
          cmx = rj*dx
c     ..............................................
c     .  calculate the number of terms needed for  .
c     .    convergence at each radius: nj .le. nc  .
c     ..............................................
          nj = min(int(abs(cmx)+4.05*abs(cmx)**.3333+2.0),nc)
          call besj(cmx,aj,nj)
          if(ip.eq.1) then
c     .....................
c     .  TM polarization  .
c     .....................
            e2 = cfa(1)*abs(aj(1))**2
            do 20 n = 2,nj
              e2 = e2+2.0*cfa(n)*abs(aj(n))**2                          eq 2.27
20          continue
          else
c     .....................
c     .  TE polarization  .
c     .....................
            e2 = cfa(1)*abs(-aj(2))**2
            do 30 n = 2,nj
              rn = real(n-1)
              t = abs(aj(n-1)-aj(n)*rn/cmx)**2
              t = cfa(n)*((rn*abs(aj(n))/abs(cmx))**2+t)
              e2 = e2+2.0*t                                             eq 2.29
30          continue
            e2 = e2/cm2
          end if
          rj = rj/cpm
          write(9,110) rj,e2
40      continue
      else
c     .............................................
c     .  calculate external integrated intensity  .
c     .............................................
        write(6,*) 'external intensity - - - enter npnts ( >1 )'
        read(5,*) npnts
        cpm = real(npnts-1)
        dtx = .5*x/cpm
        call clndr(x,cm,ip,nc,ic)
        do 50 i = 1,nc
          cfa(i) = real(cf(i))**2+aimag(cf(i))**2
50      continue
        do 80 j = 1,npnts
          rj = real(j-1)
          xtj = x+rj*dtx
c     ..............................................
c     .  calculate the number of terms needed for  .
c     .    convergence at each radius: nj .le. nc  .
c     ..............................................
          nj = int(xtj+4.05*xtj**.3333+2.0)
          call besh(xtj,bj,by,nj)
          if(ip.eq.1) then
c     .....................
c     .  TM polarization  .
c     .....................
            h = cmplx(bj(1),by(1))
            e2 = cfa(1)*abs(h)**2+bj(1)**2-2.0*bj(1)*real(cf(1)*h)
            do 60 n = 2,nj
              h = cmplx(bj(n),by(n))
              e2 = e2+2.0*(cfa(n)*abs(h)**2+bj(n)**2
     1                     -2.0*bj(n)*real(cf(n)*h))                    eq 2.31
60          continue
          else
c     .....................
c     .  TE polarization  .
c     .....................
            bjp = -bj(2)
            hp = -cmplx(bj(2),by(2))
            e2 = cfa(1)*abs(hp)**2+bjp**2-2.0*bjp*real(cf(1)*hp)
            do 70 n = 2,nj
              rn = real(n-1)
              bjp = bj(n-1)-bj(n)*rn/xtj
              h = cmplx(bj(n),by(n))
              hp = cmplx(bj(n-1),by(n-1))-h*rn/xtj
              t = cfa(n)*abs(hp)**2+bjp**2-2.0*bjp*real(cf(n)*hp)
              t = (cfa(n)*abs(h)**2+bj(n)**2
     1             -2.0*bj(n)*real(cf(n)*h))*(rn/xtj)**2+t
              e2 = e2+2.0*t                                             eq 2.32
70          continue
          end if
          rj = 1.0+.5*rj/cpm
          write(9,110) rj,e2
80      continue
      end if   
      close(unit=9)
      stop
100   format('..............................................',/,
     1       '.  calculate integrated intensity vs radius  .',/,
     2       '.  output is written to c4.dat               .',/,
     3       '..............................................',//,
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
      complex cf(329),amat(329),an,cm,cmm,b,z,h,hm,aj(350)
      common /cfcom/ cf
      dimension bj(350),by(329)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 2.18
c     .........................................................
c     .  the number of terms required for convergence of the  .
c     .    total external intensity at 1.5*x needs to be set  .
c     .    as if the cylinder radius were 1.5*x to insure     .
c     .    convergence of the incident field expansion        .
c     .........................................................
      if(ic.eq.2) xc = (1.5*x)+4.05*(1.5*x)**.3333+2.0
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
c     ........................................................
c     .  calculate the Bessel functions - the order is       .
c     .    incremented by one in the bj(*) and by(*) arrays  .
c     ........................................................
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
      dimension bj(350),by(nc)
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