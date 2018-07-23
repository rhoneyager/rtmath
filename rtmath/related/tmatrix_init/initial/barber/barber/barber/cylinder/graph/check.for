      program check
      open(unit=9,file='c5.dat')
      pi = 3.14159265358979
      read(9,*) x,y
      sum = y/2.
      read(9,*) x,y
      sum = sum+y
      dlt = x
      npnts = int(180./dlt)+1
      dphi = pi/real(npnts-1)
      do 10 i = 3,npnts-1
        read(9,*) x,y
        sum = sum+y
10    continue
      read(9,*) x,y
      sum = sum+y/2.
      sum = dphi*sum/pi
      write(6,100) sum
100   format(' sum = ',e14.6)
      close(unit=9)
      stop
      end

