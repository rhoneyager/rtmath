      dimension ab(5),r(5),a(5,5)
      data ab/.33333333,.5,1.,2.,3./
      data r/1.,2.5,5.,7.5,10./
      open(unit=9,file='ka')
      do 20 ir = 1,5
        do 10 iab = 1,5
          a(ir,iab) = r(ir)*(ab(iab))**.66666667
10      continue
20    continue
      write(9,100) (ab(i),i=1,5)
      do 30 ir = 1,5
        write(9,200) r(ir),(a(ir,i),i=1,5)
30    continue
      close(unit=9)
      stop
100   format(6x,5f6.2)
200   format(6f6.2)
      end
