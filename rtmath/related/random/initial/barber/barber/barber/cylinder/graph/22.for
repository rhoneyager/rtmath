      program twnyto
      open(unit=9,file='22.dat')
      x = 100.
      nc = int(x+4.05*x**.3333+2.0)
      cpm = 100.
      do 10 j = 1,101
        rj = real(j-1)
        xj = rj*1.1
        cnja = real(min(int(xj+4.05*xj**.3333+2.0),nc))/real(nc)
        xj = rj*1.5
        cnjb = real(min(int(xj+4.05*xj**.3333+2.0),nc))/real(nc)
        xj = rj*2.0
        cnjc = real(min(int(xj+4.05*xj**.3333+2.0),nc))/real(nc)
        rj = rj/cpm
        write(9,100) rj,cnja,cnjb,cnjc
10    continue
      do 20 j = 2,51
        rj = real(j-1)
        xtj = x+rj
        cnj = real(int(xtj+4.05*xtj**.3333+2.0))/real(nc)
        rj = 1.0+.01*rj
        write(9,110) rj,cnj
20    continue
      close(unit=9)
      stop
100   format(f6.3,3f6.3)
110   format(f6.3,18x,f6.3)
      end
