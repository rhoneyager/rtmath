      program S7
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .............................................................
c     .  calculate the internal intensity on a central line or a  . 
c     .    rectangular grid for a sphere                          .
c     .  inputs: ip = polarization parallel or perpendicular      .
c     .          x = size parameter (ka)                          .
c     .          cm = complex index of refraction, (real,imag)    .
c     .               (imag is positive for absorption)           .
c     .          npnts = number of points (multiple of 4)         .
c     .          idim = graph dimension (2 or 3)                  .
c     .                                                           .
c     .  dimension of arrays c(*) and d(*):                       .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,      .
c     .    nc = 225                                               .
c     .                                                           .
c     .  dimension of arrays bj(*), by(*), hkl(*), bsl(*), and    .
c     .    pnmllg(*): nc+1, e.g., for x = 200, nc+1 = 226         .
c     .                                                           .
c     .  arrays are set for a maximum size parameter of 200       .
c     .............................................................
      complex cm,c(225),d(225),bsl(226),er,eth,eph,b,cmx
      common /cfcom/ c,d
      dimension ac(1000),tht(2),pnmllg(226)
      pi = 3.14159265358979
      write(6,100)
      write(6,*) 'enter polarization, size parameter, mr, mi'
      read(5,*) ip,x,cmr,cmi
c     .........................................
c     .  set the complex index of refraction  .
c     .    for an exp(-iwt) time variation    .
c     .........................................
      cm = cmplx(cmr,cmi)
      write(6,110)
      write(6,*) 'enter npnts, idim'
      read(5,*) npnts,idim
      open(unit=9,file='s7.dat')
      rewind 9
c     ..................................
c     .  set npnts to a multiple of 4  .
c     ..................................
      npnts = 4*(npnts/4)
      npnt = npnts/2
      npm = npnts+1
      if(idim.eq.2) then
        ngm = 2
        write(6,120) npnts
      else
        ngm = npnts/4+1
        write(6,130) ngm,npnts
        do 10 i = 1,npnts
          ac(i) = 0.0
10      continue
        write(9,150) (ac(j),j=1,npnts)
      end if
      dltz = 2.0/real(npnts-1)
      dltx = 2.0/real(npnt)
      call sphere(x,cm,ip,nc)
c     ....................................
c     .  set starting x grid value (xg)  .
c     .  enter loop to vary x values     .
c     ....................................
      xg = real(2-idim)*(1.0-dltx)
      do 60 ig = 2,ngm
c     ...................................................
c     .  set starting z grid value (zg)                 .
c     .  ns = first grid point on or inside the sphere  .
c     .  enter loop to vary z values                    .
c     ...................................................
        ns = int((1.0-sqrt(1.0-xg**2))/dltz)+2
        if(ig.eq.ngm) ns = 1
        zg = -1.0+dltz*real(ns-1)
        do 40 jg = ns,npnt
          ra = sqrt(xg**2+zg**2)
          ta = atan(xg/zg)
          tht(1) = pi-ta
          tht(2) = ta
          cmx = ra*cm*x
c     ..............................................
c     .  calculate the number of terms needed for  .
c     .    convergence at each radius: nj .le. nc  .
c     ..............................................
          nj = min(int(abs(cmx)+4.05*abs(cmx)**.3333+2.0),nc)
          nj1 = nj+1
          call besj(cmx,bsl,nj1)
c     ...........................................
c     .  tht(1) calculates intensity for z < 0  .
c     .  tht(2) calculates intensity for z > 0  .
c     ...........................................
          do 30 ith = 1,2
            er = 0.0
            eth = 0.0
            eph = 0.0
            theta = tht(ith)
            sinth = sin(theta)
            costh = cos(theta)
            call genlgp(theta,pnmllg,nj1)
            do 20 n = 1,nj
              n1 = n+1
              rn = real(n)
              p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
              p2 = pnmllg(n1)
              b = bsl(n)-rn*bsl(n1)/cmx
c     ..................................
c     .  calculate the electric field  .
c     .    in the phi = pi plane       .
c     .    sin(phi) = sin(pi) = 0.0    .
c     .    cos(phi) = cos(pi) = -1.0   .
c     ..................................
              if(ip.eq.1) then
c     ..................................................
c     .  parallel (theta) incident polarization gives  .
c     .    radial and theta internal polarization      .
c     ..................................................
                er = er-rn*(rn+1.0)*(bsl(n1)/cmx)*p2*sinth*d(n)         eq 4.33a
                eth = eth-bsl(n1)*p2*c(n)-b*p1*d(n)                     eq 4.33b
              else
c     .........................................................
c     .  perpendicular (phi) incident polarization gives phi  .
c     .    internal polarization                              .
c     .........................................................
                eph = eph+bsl(n1)*p1*c(n)-b*p2*d(n)                     eq 4.34c
              end if
20          continue
            e2 = abs(er)**2+abs(eth)**2+abs(eph)**2                     eq 4.35
c     ...............................................
c     .  store intensity at (xg,zg) in array ac(*)  .
c     ...............................................
            if(ith.eq.1) then
              ac(jg) = e2
            else
              ac(npm-jg) = e2
            end if
30        continue
c     .......................
c     .  increment z value  .
c     .......................
          zg = zg+dltz
40      continue
c     ......................................
c     .  write out data for all zg values  .
c     .    (for given xg value)            .
c     ......................................
        if(idim.eq.2) then
          zp = -1.0
          do 50 j = 1,npnts
            write(9,140) zp,ac(j)
            zp = zp+dltz
50        continue
        else
          write(9,150) (ac(j),j=1,npnts)
        end if
c     .......................
c     .  increment x value  .
c     .......................
        xg = xg+dltx
60    continue
      close(unit=9)
      stop
100   format('...........................................',/,
     1       '.  calculate 2D or 3D internal intensity  .',/,
     2       '.  output is written to s7.dat            .',/,
     3       '...........................................',//,
     4       'polarization: parallel (1) perpendicular (2)',/,
     5       '              to the grid ( x - z ) plane',/,
     6       'size parameter: x',/,
     7       'index of refraction: real,imaginary (+ for absorption)',/)
110   format(/,'number of points: npnts (multiple of 4)',/,
     2         'graph dimension: idim (2 or 3)',/)
120   format(/,'2D dimension:',i4)
130   format(/,'3D grid dimension:',i4,' x',i4)
140   format(2e14.6)
150   format(e14.6)
      end 
      subroutine sphere(x,cm,ip,nc)
c     ..............................................................
c     .  calculate the internal field c(n) and d(n) coefficients   .
c     .    the c(n) and d(n) for theta incident polarization are,  .
c     .    within an n-dependent factor, the same as the c(n) and  .
c     .    d(n) coefficients, respectively, defined in C.F.        .
c     .    Bohren and D.R. Huffman, Absorption and Scattering of   .
c     .    Light by Small Particles (Wiley- Interscience,New       .
c     .    York,1983), p.100                                       .
c     ..............................................................
      complex cm,ci,z,hkl(226),bsl(226),hnjn,hnjnm,hnmjn,c(225),d(225)
      common /cfcom/ c,d
      ci = (0.0,1.0)
c     ......................................................
c     .  set the number of terms required for convergence  .
c     ......................................................
      xc = x+4.05*x**.3333+2.0                                          eq 4.16
      nc = int(xc)
      nci = nc+1
      z = cm*x
c     ...........................................................
c     .  calculate the Bessel functions - the order is          .
c     .    incremented by one in the hkl(*) and bsl(*) arrays  .
c     ...........................................................
      call besh(x,hkl,nci)
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
c     ........................................................
c     .  internal field coefficients for theta               .
c     .    (parallel) incident polarization                  .
c     .    c(n) = ci**n*rf*(Bohren and Huffman's c(n))       .
c     .    d(n) = -ci**(n+1)*rf*(Bohren and Huffman's d(n))  .
c     ........................................................
        c(n) = ci**n*rf*(ci/x)/((hnmjn-cm*hnjnm)*x)                     eq 4.17a
        d(n) = -ci**(n+1)*rf*(cm*ci/x)/(x*(cm**2*hnmjn-cm*hnjnm)
     1                                     -rn*(cm**2-1.0)*hnjn)        eq 4.17b
        if(ip.eq.2) then
c     ...............................................
c     .  internal field coefficients for phi        .
c     .    (perpendicular) incident polarization    .
c     ...............................................
          c(n) = -c(n)                                                  eq 4.5a
          d(n) = d(n)                                                   eq 4.5b
        end if
10    continue
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