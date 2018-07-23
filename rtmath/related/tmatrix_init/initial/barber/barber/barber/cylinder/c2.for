      program C2
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .............................................................
c     .  calculate the scattering over a finite aperture          .
c     .    by an infinite cylinder illuminated at normal          .
c     .    incidence for two cases:                               .
c     .  case 1: intensity at a specific angle vs size parameter  . 
c     .  case 2: intensity vs scattering angle                    .
c     .  inputs: ic = case                                        .
c     .          ip = polarization par (TM) or perp (TE)          .
c     .          cm = complex index of refraction, (real,imag)    .
c     .               (imag is positive for absorption)           .
c     .          x = size parameter (ka)                          .
c     .              x1 = lowest value, xn = highest value        .
c     .          ang = scattering angle, degrees                  .
c     .          dlt = increment in scattering angle,degrees      .
c     .          w = aperture width, degrees                      .
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
      complex cf(225),cm
      common /cfcom/ cf
      dimension th(2)
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
      open(unit=9,file='c2.dat')
      rewind 9
      if(ic.eq.2) then
c     .............................................
c     .  calculate intensity vs scattering angle  .
c     .............................................
        write(6,*) 'I vs angle - - - enter x, delta angle, aperture'
        read(5,*) x,dlt,w
        nang = int(180.0/dlt)+1
        dlt = dlt*dtr
        hw = w*dtr/2.0
        call clndr(x,cm,ip,nc)
        do 10 i = 1,nang
          ri = real(i-1)
          ang = ri*dlt
          th(1) = ang-hw
          th(2) = ang+hw
          call finite(th,tp,nc)
          ang = ang*rtd
          write(9,110) ang,tp
10      continue
        close(unit=9)
        stop
      end if
c     ...........................................
c     .  calculate intensity vs size parameter  .
c     ...........................................
        write(6,*) 'I vs x - - - enter x1, xn, npnts, angle, aperture'
        read(5,*) x1,xn,npnts,ang,w
        ang = ang*dtr
        hw = w*dtr/2.0
        th(1) = ang-hw
        th(2) = ang+hw
      write(6,*) 'enter delta in x (1) or inverse x (2)'
      read(5,*) id
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
      do 20 n = 1,npnts
        call clndr(x,cm,ip,nc)
        call finite(th,tp,nc)
        rn = real(n)
        if(id.eq.2) then
          xi = 1.0/x
          write(9,110) xi,tp
          x = 1.0/(rn*dwl+xni)
        else
          write(9,110) x,tp
          x = x1+rn*dk
        end if
20    continue
      close(unit=9)
      stop
100   format('.....................................................',/,
     1       '.  calculate scattered intensity - finite aperture  .',/,
     2       '.  output is written to c2.dat                      .',/,
     3       '.....................................................',//,
     4       'case: I vs x (1), I vs angle (2)',/,
     5       'polarization: TM (1) TE (2)',/,
     6       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(2e14.6)
      end
      subroutine finite(th,tp,nc)
c     .............................................................
c     .  calculate the average scattered intensity over a finite  .
c     .    aperture - integrate (closed-form) the intensity over  .
c     .    the aperture and divide by the aperture width          .
c     .............................................................
      complex cf(225)
      common /cfcom/ cf
      dimension th(2),s(2),cfr(225),cfi(225)
      do 10 i = 1,nc
        cfr(i) = real(cf(i))
        cfi(i) = aimag(cf(i))
10    continue
c     ...........................................
c     .  evaluate the closed-form integral at   .
c     .    the upper and lower limits (angles)  .
c     ...........................................
      do 40 ith = 1,2
        ang = th(ith)
        s1 = 0.0
        s2 = 0.0
        s3 = 0.0
        s4 = 0.0
        do 30 n = 2,nc
          rn = real(n-1)
          t = sin(rn*ang)/rn
          s1 = s1+cfr(n)*t
          s2 = s2+cfi(n)*t
          t = cfr(n)**2+cfi(n)**2
          s3 = s3+t*(ang/2.0+sin(2.0*rn*ang)/(4.0*rn))
          nm = n+1
          do 20 m = nm,nc
            rm = real(m-1)
            t = cfr(n)*cfr(m)+cfi(n)*cfi(m)
            s4 = s4+t*(sin((rn+rm)*ang)/(rn+rm)
     1                +sin((rn-rm)*ang)/(rn-rm))  
20        continue
30      continue
          s(ith) = 4.0*(cfr(1)*s1+cfi(1)*s2+s3+s4)                      eq 2.25
40    continue
c     ............................................
c     .  divide by the aperture width to obtain  .
c     .    the average scattered intensity       .
c     ............................................
      tp = (s(2)-s(1))/(th(2)-th(1))+cfr(1)**2+cfi(1)**2
      return
      end
      subroutine clndr(x,cm,ip,nc)
c     .......................................................
c     .  calculate the scattering coefficients - the order  .
c     .    is incremented by one                            .     
c     .......................................................
      complex cf(225),amat(225),an,cm,cmm,b,z
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
c     .......................................................
c     .  calculate the higher mode scattering coefficients  .
c     .......................................................
      do 30 n = 2,nc
        rn = real(n-1)
        b = cmm*amat(n)+rn/x
        cf(n) = (b*bj(n)-bj(n-1))/(b*cmplx(bj(n),by(n))-
     1  cmplx(bj(n-1),by(n-1)))                                         eq 2.19
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