      program rayleigh
      complex cm,tm,te
c     ......................................................
c     .  calculate test cases using the Rayleigh solution  .
c     .    for both TM and TE polarizations                .
c     .  inputs:                                           .
c     .    x = size parameter (ka)                         .
c     .    mr = real part of the index of refraction       .
c     .    mi = imaginary part of the index of refraction  .
c     ......................................................
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
      tm = cm**2-1.0
      tmabs = abs(tm)
      te = tm/(cm**2+1.0)
      teabs = abs(te)
c     .........................................
c     .  calculate the extinction efficiency  .
c     .........................................
      write(9,*) 'C1: extinction efficiency'
      t = pi*x*aimag(tm)/2.0                                            eq 2.47
      write(9,120) t
      t = pi*x*aimag(te)                                                eq 2.49
      write(9,130) t
c     .........................................
c     .  calculate the scattering efficiency  .
c     .........................................
      write(9,*) 'C1: scattering efficiency'
      t = pi**2*x**3*tmabs**2/8.0                                       eq 2.48
      write(9,120) t
      t = pi**2*x**3*teabs**2/4.0                                       eq 2.50
      write(9,130) t
c     ......................................
c     .  calculate the angular scattering  .
c     ......................................
      write(9,*) 'C1: angular scattering'  
      t = (pi*x**2*tmabs/4.0)**2                                        eq 2.51
      write(9,140) t
      phi = 0.0
        do 10 i = 1,5
        ang = 45.0*real(i-1)
        t = (pi*x**2*teabs/2.0)**2*cos(phi)**2                          eq 2.52
        write(9,150) ang,t
        phi = phi+pi/4.0
10      continue
c     ............................................
c     .  calculate the angle-averaged intensity  .
c     ............................................
      write(9,*) 'C4: angle-averaged intensity'
      t = 1.0
      write(9,160)
      write(9,140) t
      write(9,170)
      write(9,140) t
      write(9,160)
      t = (2.0/abs(cm**2+1.0))**2                                       eq 2.43
      write(9,180) t
      rad = 1.0
      t = 0.5*(abs(1.0+te)**2+abs(1.0-te)**2)                           eq 2.44
      write(9,170)
      write(9,150) rad,t      
      rad = 1.5
      aovr2 = (1.0/rad)**2
      t = 0.5*(abs(1.0+aovr2*te)**2+abs(1.0-aovr2*te)**2)               eq 2.44
      write(9,150) rad,t
c     .....................................
c     .  calculate the surface intensity  .
c     .....................................
      write(9,*) 'C5: surface intensity'
      t = 1.0
      write(9,160) 
      write(9,140) t
      write(9,170)
      write(9,140) t
      write(9,160)
      t = (2.0/abs(cm**2+1.0))**2                                       eq 2.43
      write(9,180) t
      write(9,170)
      phi = 0.0
        do 20 i = 1,5
        ang = 45.0*real(i-1)
        t = abs(1.0+te)**2*sin(phi)**2+abs(1.0-te)**2*cos(phi)**2       eq 2.45
        write(9,150) ang,t
        phi = phi+pi/4.0
20      continue
c     ...................................................
c     .  calculate the internal intensity distribution  .
c     ...................................................
      write(9,*) 'C6: internal intensity'
      t = 1.0
      write(9,140) t
      t = (2.0/abs(cm**2+1.0))**2                                       eq 2.43
      write(9,180) t
c     ...................................................
c     .  calculate the external intensity distribution  .
c     ...................................................
      write(9,*) 'C7: external intensity (npnts = 4, 3D)'
      t = 1.0
      write(9,140) t
      dltx = 5.0/3.0
      dlty = 2.5
      yg = -2.5
        do 40 j = 1,2
        xg = -2.5
          do 30 i = 1,4
          phi = atan(yg/xg)
          rad = sqrt(xg**2+yg**2)
          aovr2 = (1.0/rad)**2
          t = abs(1.0+aovr2*te)**2*sin(phi)**2
     1       +abs(1.0-aovr2*te)**2*cos(phi)**2                          eq 2.46
          if(rad.le.1.0) t = 0.0 
          write(9,130) t
          xg = xg+dltx
30        continue
        yg = yg+dlty
40      continue
      close(unit=9)      
      stop
100   format('................................................',/,
     1       '.  calculate the small-particle approximation  .',/,
     2       '.  output is written to ray.dat                .',/,
     3       '................................................',/)
110   format('small-particle solution: x =',f6.4,' m =',f5.2,',',f5.2,/)
120   format(5x,'TM:',e14.6)
130   format(5x,'TE:',e14.6)
140   format(5x,'TM:',e14.6,' (constant)')
150   format(5x,'TE:',f8.1,e14.6)
160   format(5x,'internal')
170   format(5x,'external')
180   format(5x,'TE:',e14.6,' (constant)')
      end
