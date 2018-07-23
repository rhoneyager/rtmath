      program C1
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .............................................................
c     .  calculate the scattering by an infinite cylinder         .
c     .    illuminated at normal incidence for three cases:       .
c     .  case 1: efficiency (Qe, Qs and Qa) vs size parameter     .
c     .  case 2: intensity at a specific angle vs size parameter  . 
c     .  case 3: intensity vs scattering angle                    .
c     .  inputs: ic = case                                        .
c     .          ip = polarization par (TM) or perp (TE)          .
c     .          cm = complex index of refraction, (real,imag)    .
c     .               (imag is positive for absorption)           .
c     .          x = size parameter (ka)                          .
c     .              x1 = lowest value, xn = highest value        .
c     .          ang = scattering angle, degrees                  .
c     .          dlt = increment in scattering angle, degrees     .
c     .          npnts = number of points                         .
c     .                                                           .
c     .  dimension of arrays cf(*), amat(*) and by(*):            .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,      .
c     .    nc = 225                                               .
c     .                                                           .
c     .  dimension of array bj(*):                                .
c     .    nst+1 = nc+int((101.0+x)**.5)+1, e.g., for x = 200,    .
c     .    nst+1 = 243                                            .
c     .                                                           .
c     .  arrays are set for a maximum size parameter of 200       .
c     .............................................................
      complex cf(225),cm,t
      common /cfcom/ cf
      pi = 3.14159265358979
      rtd = 180.0/pi
      dtr = 1.0/rtd
      write(6,100)
      write(6,*) 'enter case, polarization, mr, mi'
      read(5,*) ic,ip,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      open(unit=9,file='c1.dat')
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
        ang = 0.0
        do 20 i = 1,npnts
          call clndr(x,cm,ip,nc,ang,t)
c     .........................................
c     .  calculate the extinction efficiency  .
c     .........................................
          qext = 2.0*real(t)/x                                          eq 2.15
c     ..........................................
c     .   calculate the scattering efficiency  .
c     ..........................................
          qsca = abs(cf(1))**2
          do 10 n = 2,nc
            qsca = qsca+2.0*abs(cf(n))**2                               eq 2.16
10        continue
          qsca = 2.0*qsca/x
c     .........................................
c     .  calculate the absorption efficiency  .
c     .........................................
          qabs = qext-qsca                                              eq 2.17
          ri = real(i)
          if(id.eq.1) then
            write(9,110) x,qext,qsca,qabs
            x = x1+ri*dk
          else
            xi = 1.0/x
            write(9,110) xi,qext,qsca,qabs
            x = 1.0/(ri*dwl+xni)
          end if
20      continue
      else if(ic.eq.2) then
c     ...........................................
c     .  calculate intensity vs size parameter  .
c     ...........................................
        write(6,*) 'I vs x - - - enter x1, xn, npnts, angle'
        read(5,*) x1,xn,npnts,ang
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
        ang = ang*dtr
        do 30 i = 1,npnts
          call clndr(x,cm,ip,nc,ang,t)
          tp = abs(t)**2
          ri = real(i)
          if(id.eq.1) then
            write(9,120) x,tp
            x = x1+ri*dk
          else
            xi = 1.0/x
            write(9,120) xi,tp
            x = 1.0/(ri*dwl+xni)
          end if
30      continue
        else
c     .............................................
c     .  calculate intensity vs scattering angle  .
c     .............................................
        write(6,*) 'I vs angle - - - enter x, delta angle'
        read(5,*) x,dlt
        nang = int(180.0/dlt)+1
        dlt = dlt*dtr
        ang = 0.0
        call clndr(x,cm,ip,nc,ang,t)
        tp = abs(t)**2
        write(9,110) ang,tp
        do 50 i = 2,nang
          ri = real(i-1)
          ang = ri*dlt
          t = cf(1)
          do 40 n = 2,nc
            rn = real(n-1)
            t = t+2.0*cf(n)*cos(rn*ang)                                 eq 2.24
40        continue
          tp = abs(t)**2
          ang = ang*rtd
          write(9,120) ang,tp
50      continue
      end if
      close(unit=9)
      stop
100   format('...................................................',/,
     1       '.  calculate efficiencies or scattered intensity  .',/,
     2       '.  output is written to c1.dat                    .',/,
     3       '...................................................',//,
     4       'case: Q vs x (1), I vs x (2), I vs angle (3)',/,
     5       'polarization: TM (1) TE (2)',/,
     6       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(4e14.6)
120   format(2e14.6)
      end
      subroutine clndr(x,cm,ip,nc,ang,t)
c     .......................................................
c     .  calculate the scattering coefficients - the order  .
c     .    is incremented by one                            .
c     .  t is the scattering amplitude at angle ang         .
c     .    (when ang = zero, this is the same as the sum    .
c     .     required in the calculation of the extinction   .
c     .     efficiency)                                     .
c     .......................................................
      complex cf(225),amat(225),an,cm,cmm,b,t,z
      common /cfcom/ cf
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
c     ........................................................
c     .  calculate the Bessel functions - the order is       .
c     .    incremented by one in the bj(*) and by(*) arrays  .
c     ........................................................
      call besh(x,bj,by,nc)
c     .......................................................
c     .  calculate the zeroeth mode scattering coefficient  .
c     .......................................................
      cmm = cm
      if(ip.eq.2) cmm = 1.0/cm
      cf(1) = (cmm*amat(1)*bj(1)+bj(2))/(cmm*amat(1)*cmplx(bj(1),by(1))
     1+cmplx(bj(2),by(2)))
      t = cf(1)
c     .......................................................
c     .  calculate the higher mode scattering coefficients  .
c     .......................................................
      do 30 n = 2,nc
        rn = real(n-1)
        b = cmm*amat(n)+rn/x
        cf(n) = (b*bj(n)-bj(n-1))/(b*cmplx(bj(n),by(n))-
     1  cmplx(bj(n-1),by(n-1)))                                         eq 2.19
        t = t+2.0*cf(n)*cos(rn*ang)
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
