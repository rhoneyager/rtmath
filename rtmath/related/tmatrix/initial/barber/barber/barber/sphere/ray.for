      program rayleigh
      complex cm,ratio
c     .........................................................
c     .  calculate test cases using the Rayleigh solution     .
c     .    for both parallel and perpendicular polarizations  .
c     .  inputs:                                              .
c     .    x = size parameter (ka)                            .
c     .    mr = real part of the index of refraction          .
c     .    mi = imaginary part of the index of refraction     .
c     .........................................................
      pi = 3.14159265358979
c     ..................................................
c     .  enter size parameter and index of refraction  .
c     ..................................................
      write(6,100)
      write(6,*) 'enter x ( .le. 0.01 recommended ), mr, and mi'
      read(5,*) x,cmr,cmi
      open(unit=9,file='ray.dat')
      rewind 9
      write(9,110) x,cmr,cmi
      cm = cmplx(cmr,cmi)
      ratio = (cm**2-1.)/(cm**2+2.)
c     .........................................
c     .  calculate the extinction efficiency  .
c     .........................................
      write(9,*) 'S1: extinction efficiency'
      t = 4.*x*aimag(ratio*(1.+(x**2/15.)*ratio
     1*(cm**4+27.*cm**2+38.)/(2.*cm**2+3.)))
     2+(8./3.)*x**4*real(ratio**2)                                      eq 4.61
      write(9,120) t
c     .........................................
c     .  calculate the scattering efficiency  .
c     .........................................
      write(9,*) 'S1: scattering efficiency'
      t = (8./3.)*x**4*abs(ratio)**2                                    eq 4.62
      write(9,120) t
c     ......................................
c     .  calculate the angular scattering  .
c     ......................................
      write(9,*) 'S1: angular scattering'  
      theta = 0.
      do 10 i = 1,5
        ang = 45.*real(i-1)
        t = (x**4/pi)*abs(ratio)**2*cos(theta)**2                       eq 4.63 
        write(9,130) ang,t
        theta = theta+pi/4.
10    continue
      t = (x**4/pi)*abs(ratio)**2                                       eq 4.64
      write(9,140) t
c     ................................................
c     .  calculate the scattering in all directions  .
c     ................................................
      write(9,*) 'S2: angular scattering (npnts = 4, 3D)'
      dlt = 2./3.
      bkscat = (x**4/pi)*abs(ratio)**2
      yg = -1.
      do 30 ig = 1,4
        xg = -1.
        do 20 jg = 1,4
          r = sqrt(xg**2+yg**2)
          theta = r*pi
          phi = atan2(yg,xg)
          sinph = sin(phi)
          cosph = cos(phi)
          costh = cos(theta)
          tpar = bkscat*(cosph**2*costh**2+sinph**2)                    eq 4.65
          if(r.ge.1) tpar = bkscat
          tperp = bkscat*(sinph**2*costh**2+cosph**2)                   eq 4.66
          if(r.ge.1) tperp = bkscat
          tpar = alog10(tpar)
          tperp = alog10(tperp)
          write(9,150) tpar,tperp
          xg = xg+dlt
20      continue
        yg = yg+dlt
30    continue
c     .....................................................
c     .  calculate the elements of the scattering matrix  .
c     .....................................................
      write(9,*) 'S3: scattering matrix'
      write(9,*) '                 p11      pl = -p12/p11    p33/p11'
      theta = 0.
      do 40 i = 1,5
        ang = 45.*real(i-1)
        p11 = 3.*(cos(theta)**2+1.)/4.                                  eq 4.67a
        pl = -(cos(theta)**2-1.)/(cos(theta)**2+1.)                     eq 4.67b
        p33p11 = 2.*cos(theta)/(cos(theta)**2+1.)                       eq 4.67c
        write(9,160) ang,p11,pl,p33p11
        theta = theta+pi/4.
40    continue
c     ............................................
c     .  calculate the angle-averaged intensity  .
c     ............................................
      write(9,*) 'S5: angle-averaged intensity'
      tc = abs(3./(cm**2+2.))**2                                        eq 4.54
      write(9,170)
      write(9,180) tc
      rad = 1.
      t = abs(2.*ratio+1.)**2/3.+2.*abs(ratio-1.)**2/3.                 eq 4.55
      write(9,190)
      write(9,200) rad,t      
      rad = 1.5
      aovr3 = (1./rad)**3
      t = abs(2.*ratio*aovr3+1.)**2/3.+2.*abs(ratio*aovr3-1.)**2/3.     eq 4.55
      write(9,200) rad,t
c     .....................................
c     .  calculate the surface intensity  .
c     .....................................
      write(9,*) 'S6: surface intensity'
      write(9,170) 
      write(9,210) tc                                                   eq 4.56
      write(9,190)
      theta = 0.
      do 50 i = 1,5
        ang = 45.*real(i-1)
        t = abs(3.*cm**2/(cm**2+2.))**2*sin(theta)**2
     1     +abs(3./(cm**2+2.))**2*cos(theta)**2                         eq 4.57
        write(9,130) ang,t
        theta = theta+pi/4.
50    continue
      write(9,170)
      write(9,140) tc
      write(9,190)
      write(9,140) tc
c     ...................................................
c     .  calculate the internal intensity distribution  .
c     ...................................................
      write(9,*) 'S7: internal intensity'
      write(9,210) tc                                                   eq 4.58
      write(9,140) tc                                                   eq 4.58
c     ...................................................
c     .  calculate the external intensity distribution  .
c     ...................................................
      write(9,*) 'S8: external intensity (npnts = 4, 3D)'
      dltz = 5./3.
      dltx = 2.5
      xg = -2.5
      do 70 j = 1,2
        zg = -2.5
        do 60 i = 1,4
          theta = atan(xg/zg)
          rad = sqrt(xg**2+zg**2)
          aovr3 = (1./rad)**3
          tpar = abs(2.*ratio*aovr3+1.)**2*sin(theta)**2
     1          +abs(ratio*aovr3-1.)**2*cos(theta)**2                   eq 4.59
          if(rad.le.1.) tpar = 0. 
          tperp = abs(ratio*aovr3-1.)**2                                eq 4.60
          if(rad.le.1.) tperp = 0.
          write(9,150) tpar,tperp
          zg = zg+dltz
60      continue
        xg = xg+dltx
70    continue
      close(unit=9)      
      stop
100   format('................................................',/,
     1       '.  calculate the small-particle approximation  .',/,
     2       '.  output is written to ray.dat                .',/,
     3       '................................................',/)
110   format('small-particle solution: x =',f6.4,' m =',f5.2,',',f5.2,/)
120   format(3x,e14.6)
130   format(5x,'parallel:',f8.1,e14.6)
140   format(5x,'perpendicular:',e14.6,' (constant)')
150   format(5x,'parallel:',e14.6,' perpendicular:',e14.6)
160   format(2x,f8.1,3e14.6)
170   format(5x,'internal')
180   format(3x,e14.6,' (constant)')
190   format(5x,'external')
200   format(f8.1,e14.6)
210   format(5x,'parallel:',e14.6,' (constant)')
      end
