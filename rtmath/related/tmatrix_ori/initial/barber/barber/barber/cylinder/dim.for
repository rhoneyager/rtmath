      program dimension
      complex cm,z
c     ...............................................
c     .  calculate the maximum required dimensions  .
c     .    for all arrays in all subroutines        .
c     ...............................................
      write(6,100)
      write(6,*) 'enter maximum size parameter'
      read(5,*) x
      write(6,*) 'enter maximum index of refraction: real,imaginary'
      read(5,*) cmr,cmi
      cm = cmplx(cmr,cmi)
      open(unit=9,file='dim.dat')
      rewind 9
      write(9,*) '*** maximum array dimensions ***'
      write(9,110) x,cmr,cmi      
c     ........................................
c     .  calculate maximum array dimensions  .
c     ........................................
      xc = x+4.05*x**.3333+2.0
      nc = int(xc)
      nst1 = nc+int((101.0+x)**.5)+1
c     ....................
c     .  for program C1  .
c     ....................
      write(9,*) 'program C1:'
      write(9,120) nc
      write(9,140) nst1
c     ....................
c     .  for program C2  .
c     ....................
      write(9,*) 'program C2:'
      write(9,120) nc
      write(9,140) nst1
c     ....................
c     .  for program C3  .
c     ....................
      z = cm*x
      nst1a = int(abs(z)+4.05*abs(z)**.3333+2.0+(101.0+abs(z))**.5)+1
      write(9,*) 'program C3:'
      write(9,120) nc
      write(9,140) nst1
      write(9,150) nst1a
c     ....................
c     .  for program C4  .
c     ....................
      nca = int((1.5*x)+4.05*(1.5*x)**.3333+2.0)
      nst1b = nca+int((101.0+(1.5*x))**.5)+1
      write(9,*) 'program C4:'
      write(9,130) nca
      write(9,140) nst1b
      write(9,150) nst1a
c     ....................
c     .  for program C5  .
c     ....................
      write(9,*) 'program C5:'
      write(9,120) nc
      write(9,140) nst1
c     ....................
c     .  for program C6  .
c     ....................
      write(9,*) 'program C6:'
      write(9,120) nc
      write(9,140) nst1
      write(9,150) nst1a
c     ....................
c     .  for program C7  .
c     ....................
      nst1c = int((2.5*x)+4.05*(2.5*x)**.3333+2.0+(101.+(2.5*x))**.5)+1
      write(9,*) 'program C7:'
      write(9,120) nc
      write(9,140) nst1c
      close(unit=9)
      stop
100   format('......................................',/,
     1       '.  calculate the program dimensions  .',/,
     2       '.  output is written to dim.dat      .',/,
     3       '......................................',/)
110   format(/,'x = ',f8.2,'  m =',f5.2,',',f5.2,/)
120   format(/,' cf(*), amat(*), by(*): ',i4,/)
130   format(/,' cf(*), cfa(*), amat(*), by(*): ',i4,/)
140   format(' bj(*): ',i4,/)
150   format(' aj(*): ',i4,/)
      end
