      program S8
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     ............................................................
c     .  calculate the total (incident + scattered) intensity    .
c     .    on a central line or a rectangular grid for a sphere  .
c     .  inputs: ip = polarization parallel or perpendicular     .
c     .          x = size parameter (ka)                         .
c     .          cm = complex index of refraction, (real,imag)   .
c     .               (imag is positive for absorption)          .
c     .          npnts = number of points (multiple of 4)        .
c     .          idim = graph dimension (2 or 3)                 .
c     .                                                          .
c     .  dimension of arrays f(*), g(*), amat(*) and cnrm(*):    .
c     .    nc = int(x+4.05*x**.3333+2.0), e.g., for x = 200,     .
c     .    nc = 225                                              .
c     .                                                          .
c     .  dimension of arrays bj(*), by(*), hkl(*), and           .
c     .    pnmllg(*): nc+1, e.g., for x = 200, nc+1 = 226        .
c     .                                                          .
c     .  arrays are set for a maximum size parameter of 200      .
c     ............................................................
      complex cm,f(225),g(225),hkl(226),er,eth,eph,ci,b,amag
      common /cfcom/ f,g,cnrm
      dimension ac(1000),tht(2),pnmllg(226),cnrm(225)
      pi = 3.14159265358979 
      ci = (0.0,1.0)
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
      open(unit=9,file='s8.dat')
      rewind 9
c     ..................................
c     .  set npnts to a multiple of 4  .
c     ..................................
      npnts = 4*(npnts/4)
      npnt = npnts/2
      npm = npnts+1
      if(idim.eq.2) then
        ngm = 1
        write(6,120) npnts
      else
        ngm = npnts/4+1
        write(6,130) ngm,npnts
      end if
      dltz = 5.0/real(npnts-1)
      dltx = 5.0/real(npnt)
      call sphere(x,cm,ip,nc)
      nci = nc+1
c     ....................................
c     .  set starting x grid value (xg)  .
c     .  enter loop to vary x values     .
c     ....................................
      xg = real(2-idim)*2.5
      do 60 ig = 1,ngm
c     ...................................................
c     .  set starting z grid value (zg)                 .
c     .  ns = last grid point on or outside the sphere  .
c     .  enter loop to vary z values                    .
c     ...................................................
        if(xg.gt.(-1.0)) then
          ns = int((2.5-sqrt(1.0-xg**2))/dltz)+1
          do 10 i = 1,npnts
            ac(i) = 0.0
10        continue
        else
          ns = npnt
        end if
        zg = -2.5
        do 40 jg = 1,ns
          ra = sqrt(xg**2+zg**2)
          ta = atan(xg/zg)
          tht(1) = pi-ta
          tht(2) = ta
          xj = ra*x
          call besh(xj,hkl,nci)
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
            call genlgp(theta,pnmllg,nci)
            do 20 n = 1,nc
              n1 = n+1
              rn = real(n)
              p1 = rn*costh*pnmllg(n1)-(rn+1.0)*pnmllg(n)
              p2 = pnmllg(n1)
              b = hkl(n)-rn*hkl(n1)/xj
c     ..................................
c     .  calculate the electric field  .
c     .    in the phi = pi plane       .
c     .    sin(phi) = sin(pi) = 0.0    .
c     .    cos(phi) = cos(pi) = -1.0   .
c     ..................................
              if(ip.eq.1) then
c     ..................................................
c     .  parallel (theta) incident polarization gives  .
c     .    radial and theta scattered polarization     .
c     ..................................................
                er = er-rn*(rn+1.0)*(hkl(n1)/xj)*p2*sinth*g(n)
     1                                                    *cnrm(n)      eq 4.37a
                eth = eth-(hkl(n1)*p2*f(n)+b*p1*g(n))*cnrm(n)           eq 4.37b
              else
c     .........................................................
c     .  perpendicular (phi) incident polarization gives phi  .
c     .    scattered polarization                             .
c     .........................................................
                eph = eph+(hkl(n1)*p1*f(n)-b*p2*g(n))*cnrm(n)           eq 4.39c
              end if
20          continue
c     ......................................................
c     .  add the incident field to obtain the total field  .
c     ......................................................
            amag = exp(ci*xj*costh)
            if(ip.eq.1) then                                            add
              er = er-amag*sinth                                        eq 4.36
              eth = eth-amag*costh                                      eq 4.36
              e2 = abs(er)**2+abs(eth)**2                               eq 4.35
            else                                                        add
              eph = eph-amag                                            eq 4.38
              e2 = abs(eph)**2                                          eq 4.35
            end if
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
          zp = -2.5
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
     1       '.  calculate 2D or 3D external intensity  .',/,
     2       '.  output is written to s8.dat            .',/,
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
      nst = int(x+4.05*x**.3333+2.0+(101.0+x)**.5)                      eq 4.21
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