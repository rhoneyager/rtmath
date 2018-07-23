      program dimension
c     ...............................................
c     .  calculate the maximum required dimensions  .
c     .    for all arrays in all subroutines        .
c     ...............................................
      write(6,*) 'enter maximum size parameter'
      read(5,*) x
      open(unit=9,file='dim.dat')
      rewind 9
      write(9,*) '*** maximum array dimensions ***'
      write(9,110) x      
c     ........................................
c     .  calculate maximum array dimensions  .
c     ........................................

      nc = int(x+4.05*x**.3333+2.0)
      nci = nc+1
c     ....................
c     .  for program S1  .
c     ....................
      write(9,*) 'program S1:'
      write(9,120) nc
      write(9,130) nci
c     ....................
c     .  for program S2  .
c     ....................
      write(9,*) 'program S2:'
      write(9,120) nc
      write(9,130) nci
c     ....................
c     .  for program S3  .
c     ....................
      write(9,*) 'program S3:'
      write(9,120) nc
      write(9,130) nci
c     ....................
c     .  for program S4  .
c     ....................
      write(9,*) 'program S4:'
      write(9,140) nc
      write(9,150) nci
c     ....................
c     .  for program S5  .
c     ....................
      nca = int((1.5*x)+4.05*(1.5*x)**.3333+2.0)
      nca2 = nca+2
      nc2 = nc+2
      write(9,*) 'program S5:'
      write(9,160) nca
      write(9,170) nca2
      write(9,180) nc2
c     ....................
c     .  for program S6  .
c     ....................
      write(9,*) 'program S6:'
      write(9,160) nc
      write(9,190) nci
c     ....................
c     .  for program S7  .
c     ....................
      write(9,*) 'program S7:'
      write(9,200) nc
      write(9,190) nci
c     ....................
c     .  for program S8  .
c     ....................
      write(9,*) 'program S8:'
      write(9,120) nc
      write(9,130) nci
      close(unit=9)
      stop
100   format('......................................',/,
     1       '.  calculate the program dimensions  .',/,
     2       '.  output is written to dim.dat      .',/,
     3       '......................................',/)
110   format(/,'x = ',f8.2,/)
120   format(/,' f(*), g(*), amat(*), cnrm(*): ',i4,/)
130   format(' bj(*), by(*), hkl(*), pnmllg(*): ',i4,/)
140   format(/,' amat(*): ',i4,/)
150   format(' bj(*), by(*), hkl(*), bsl(*): ',i4,/)
160   format(/,' cf(*), dg(*), amat(*): ',i4,/)
170   format(' bj(*), by(*), hkl(*): ',i4,/)
180   format(' bsl(*): ',i4,/)
190   format(' bj(*), by(*), hkl(*), bsl(*), pnmllg(*): ',i4,/)
200   format(/,' c(*), d(*):',i4,/)
      end
