      program nc
      open(unit=9,file='nc.dat')
      do 10 j = 2,11
        x = 1.*real(j-1)
        nct = int(x+4.05*x**.3333+2.0)
        write(9,*) x,nct
10    continue
      close(unit=9)
      stop
      end
