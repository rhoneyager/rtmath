      program grid
      open(unit=9,file='grid.dat')
      dx = .01
      do 20 i = 1,101
        x =- real(i-1)*dx
        y = -sqrt(1.-x**2)
        write(9,100) x,y
20    continue
      do 30 i = 100,1,-1
        x = -real(i-1)*dx
        y = sqrt(1.-x**2)
        write(9,100) x,y
30    continue
      close(unit=9)
100   format(2e14.6)
      stop
      end
