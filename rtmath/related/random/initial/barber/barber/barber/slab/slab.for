      Program SLAB
c     ..................................................................
c     .  Light Scattering by Particles: Computational Methods          .
c     .  by P.W. Barber and S.C. Hill                                  .
c     .  copyright (c) 1990 by World Scientific Publishing Co Pte Ltd  .
c     .                                                                .
c     .  equation numbers in columns 73-80 are references to the text  .
c     ..................................................................
c     .  Computes reflected, transmitted and internal intensities   .
c     .    for a slab.                                              .
c     .  An exp(-i w t) time variation is assumed.                  .
c     .  The slab extends from z = 0 to z = 2a.                     .
c     .  a = half thickness of slab                                 .
c     .  x  = size parameter = 2 pi a / wavelength-outside-slab     .
c     .  m  = m' + i m'' = complex index of refraction of slab      .
c     .      The index of refraction outside the slab is 1.0        .
c     .  e1p = incident field amplitude coefficient = (1.0,0.0)     .
c     .  e1m = coefficient of reflected wave                        .
c     .  e2p = coefficient of positive travelling wave inside slab  .
c     .  e2m = coefficient of negative travelling wave inside slab  .
c     .  e3p = coefficient of transmitted wave                      .
c     ...............................................................
      complex e1p,e1m,e2p,e2m,e3p,m,i,e,ei4mx,denom

      i   = cmplx(0.0, 1.0)
      e1p = cmplx(1.0, 0.0)
      e1max = e1p * conjg(e1p) 

      open(unit=9,file='slab.dat')
      rewind 9
      write(6,100)
      write(6,*) 'enter case, mr, mi'
      read(5,*) itype,rmr,rmi
c     .........................................
c     .  Set the complex index of refraction  .
c     .    for an exp(-iwt) time variation.   .
c     .........................................
      m = cmplx(rmr,rmi)

      if ( itype .eq. 1 ) then
c       ...................................................
c       .  Compute reflected and transmitted intensities  .
c       .        vs size parameter.                       .
c       ...................................................
        write(6,*) 'enter: xmin, xmax, number of points'
        read(5,*) xmin,xmax,n
        dx = 0.0
        if ( n .gt. 1 ) dx = ( xmax - xmin ) / ( n - 1 )

        do 10 j = 1,n
          x = xmin + ( j - 1 ) * dx
c         ..............................................
c         .  Compute the electric field coefficients.  .
c         ..............................................
          ei4mx = exp( i * 4.0 * m * x )
          denom = (m * m + 1) * (ei4mx - 1.0) - 2.0 * m * (ei4mx + 1)
          e1m   = e1p * ( m * m - 1.0 ) * ( 1.0 - ei4mx ) / denom        eq 1.7
          e3p   = e1p * (-4.0) * m * exp(i * 2.0 * x * (m-1.0) ) / denom eq 1.8
c         ..........................
c         .  Relative Intensities  .
c         ..........................
          eerefl = e1m * conjg(e1m) / e1max
          eetrns = e3p * conjg(e3p) / e1max
          write(9,110) x,eerefl,eetrns
10      continue   

      elseif ( itype .eq. 2 ) then
c       ............................................................
c       .  Compute internal intensity vs position inside the slab. .
c       ............................................................
        write(6,*) 'enter: size parameter, number of points ( > 1 )'
        read(5,*) x,n
        dr = 2.0 * x / ( n - 1 )
c       ..............................................
c       .  Compute the electric field coefficients.  .
c       ..............................................
        ei4mx = exp( i * 4.0 * m * x )
        denom = (m * m + 1) * (ei4mx - 1.0) - 2.0 * m * (ei4mx + 1)
        e2p   = e1p * (-2.0) * ( m + 1.0 ) / denom                       eq 1.5
        e2m   = e1p * (-2.0) * ei4mx * ( m - 1.0 ) / denom               eq 1.6

        do 20 j = 1,n
          r = ( j - 1 ) * dr
c         ..........................
c         .  Relative Intensities  .
c         ..........................
          e   = e2p * exp (i*m*r) + e2m * exp (-i*m*r)
          ee  = e * conjg(e) / e1max
          rnorm = r / x
          write(9,120) rnorm,ee
20      continue   

      endif

      close(unit=9)
      stop
100   format('....................................................',/,
     1       '.  calculate reflected, transmitted and internal   .',/,
     2       '.     intensities for a slab                       .',/,
     3       '.  output is written to slab.dat                   .',/,
     4       '....................................................',//,
     5       'case (1): reflected and transmitted I vs size parameter',/
     6       'case (2): internal I vs distance across slab  ',/
     7       'index of refraction: real,imaginary (+ for absorption)',/
     8       '                     real = mr, imaginary = mi        ',/)
110   format(3e14.6)
120   format(2e14.6)
      end
