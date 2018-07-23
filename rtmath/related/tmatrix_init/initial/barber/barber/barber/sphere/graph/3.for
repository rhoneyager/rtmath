      program three
      open(unit=9,file='3.dat')
      do 10 j = 1,201
        x = real(j-1)
        nc = int(x+4.05*x**.3333+2.0)
        nst = nc+int((101.0+x)**.5)
        write(9,100) x,x,nc,nst
10    continue
      close(unit=9)
      stop
100   format(2f6.0,2i6)
      end
