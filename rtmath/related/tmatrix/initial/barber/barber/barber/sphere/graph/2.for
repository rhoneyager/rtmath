      program two
      open(unit=9,file='2.dat')
      do 10 j = 1,201
        x = real(j-1)
        xc = x+4.05*x**.3333+2.0
        nc = int(xc)
        nmxa = int(max(xc,1.1*x))+15
        nmxb = int(max(xc,1.5*x))+15
        nmxc = int(max(xc,2.0*x))+15
        write(9,100) x,x,nc,nmxa,nmxb,nmxc
10    continue
      close(unit=9)
      stop
100   format(2f6.0,4i6)
      end
