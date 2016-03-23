#pragma once

// Linear least-squares fitting routine, implemented by John Burkhardt at FSU.
// LGPL licence, https://people.sc.fsu.edu/~jburkardt/cpp_src/llsq/llsq.html

void llsq ( int n, double x[], double y[], double &alpha, double &beta );

