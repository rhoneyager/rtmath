      program dimension
c     ....................................................
c     .  specify the dimensions of all nrank-dependent   .
c     .    arrays in programs t1.for through t9.for      .
c     .  simple dependencies are shown in the following  .
c     .    comment block - the dependencies of arrays    .
c     .    cstore(*)  and tstore(*) are calculated in    .
c     .    the FORTRAN program which follows             .
c     ....................................................
c     ..................................................................
c     .  array          dimension                   program            .
c     .  ------   ---------------------    --------------------------  .
c     .  a        ( 2*nrank , 2*nrank )    t1                t7 t8     .
c     .  aa       ( 2*nrank , 2*nrank )    t1                          .
c     .  abg1     ( 2*nrank )                                t7        .
c     .  abg2     ( 2*nrank )                                t7        .
c     .  ab1      ( 2*nrank )              t1 t2 t3 t4 t5 t6    t8 t9  .
c     .  ab2      ( 2*nrank )              t1 t2 t3 t4 t5 t6    t8 t9  .
c     .  ab1      ( 2*nrank , nrank+1 )                      t7        .
c     .  ab2      ( 2*nrank , nrank+1 )                      t7        .
c     .  asc      ( 4*nrank )              t1                t7        .
c     .  b        ( 2*nrank , 2*nrank )    t1                t7        .
c     .  bb       ( 2*nrank , 2*nrank )    t1                          .
c     .  bj       ( nrank+1 )              t1                t7    t9  .
c     .  bslc     ( nrank+1 , 4*nrank )                      t7        .
c     .  bslcmp   ( nrank+1 )              t1                t7 t8     .
c     .  by       ( nrank+1 )              t1                t7    t9  .
c     .  cd1      ( 2*nrank, nrank+1 )                          t8     .
c     .  cd2      ( 2*nrank, nrank+1 )                          t8     .
c     .  clrmtx   ( 4*(2*nrank)**2 )       t1                t7        .
c     .  cmx      ( nrank )                      t3                t9  .
c     .  cmx      ( nrank, nrank+1 )                         t7        .
c     .  cmxnrm   ( nrank )                t1 t2    t4 t5 t6           .
c     .  cmxnrm   ( nrank , nrank+1 )            t3                t9  .
c     .  cstore   ( see program below )             t4 t5 t6           .
c     .  fg1      ( 2*nrank )              t1 t2    t4 t5 t6 t7        .
c     .  fg2      ( 2*nrank )              t1 t2    t4 t5 t6 t7        .
c     .  fg1      ( 2*nrank , nrank+1 )          t3                t9  .
c     .  fg2      ( 2*nrank , nrank+1 )          t3                t9  .
c     .  hank     ( nrank+1 , 4*nrank )                      t7        .
c     .  hankel   ( nrank+1 )              t1                t7    t9  .
c     .  ls       ( 2*nrank )              t1                t7 t8     .
c     .  p        ( nrank+1 , nrank+1 )                         t8 t9  .
c     .  pnmllg   ( nrank+1 )              t1 t2 t3 t4 t5 t6 t7 t8 t9  .
c     .  tmat     ( 2*nrank , 2*nrank )    t1 t2 t3 t4 t5 t6       t9  .
c     .  tstore   ( see program below )             t4 t5 t6           .
c     .  wt       ( 4*nrank )              t1                t7        .
c     ..................................................................
      write(6,*) 'enter nrank'
      read(5,*) nrank
      nsum1 = 2*nrank
      nsum2 = 2*((2*nrank)*(2*nrank))
      do 10 m = 2,nrank
        ns = nrank-m+1
        nsum1 = nsum1+ns
        nsum2 = nsum2+(2*ns)*(2*ns)
10    continue
      write(6,100) nsum1,nsum2
      write(6,200)
100   format('cstore(*) dimension =',i8,/,
     1       'tstore(*) dimension =',i8)
200   format(/,'see dim.for listing for other',/,
     1         'nrank-dependent array dimensions')
      stop
      end
