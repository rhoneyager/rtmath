      program norm
c     .............................................................
c     .  normalize the output of program t7 for Figs. 19 to 21    .
c     .                                                           .
c     .  horizontal axis - change the size parameter ka of the    .
c     .  spheroid to the size parameter of the volume-equivalent  .
c     .  sphere                                                   .
c     .                                                           .
c     .  vertical axis - change the normalization of the          .
c     .  scattering cross section from pi*a**2, where a is the    .
c     .  semidimension along the axis of revolution of the        .
c     .  spheroid (standard output from program t7), to pi*a**2,  .
c     .  where a is the radius of the volume-equivalent sphere    .
c     .............................................................
      dimension xs(1001),q1(1001),q2(1001)
c     ....................
c     .  set a/b = 1.05  .
c     ....................
      aovrb = 1.05
      open(unit=9,file='t7.dat')
c     ............................................................
c     .  determine if t7.dat is for fixed or random orientation  .
c     .    fixed = 3 data columns; random = two data columns     .
c     ............................................................
      ior = 1
      read(9,100) x,f1,f2
      if(f2.eq.0.0) ior = 2
      rewind 9
c     ...................................
c     .  ior = 1 for fixed orientation  .
c     ...................................
      if(ior.eq.1) then
        npnts = 0
        do 30 i = 1,1001
          read(9,100,end=40) x,f1,f2
          xs(i) = x/aovrb**(2./3.)
          q1(i) = f1*(x/xs(i))**2
          q2(i) = f2*(x/xs(i))**2 
          npnts = npnts+1
30      continue
40      rewind 9
        write(9,100) (xs(i),q1(i),q2(i),i=1,npnts) 
c     ....................................
c     .  ior = 2 for random orientation  .
c     ....................................
      else
        npnts = 0
        do 50 i = 1,1001
          read(9,110,end=60) x,f1
          xs(i) = x/aovrb**(2./3.)
          q1(i) = f1*(x/xs(i))**2
          npnts = npnts+1
50      continue
60      rewind 9
        write(9,110) (xs(i),q1(i),i=1,npnts) 
      end if
      close(unit=9)
      stop
100   format(3e14.6)
110   format(2e14.6)
      end
