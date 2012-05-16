#include "../rtmath/Stdafx.h"
#include <cmath>
#include <complex>
#include <boost/math/constants/constants.hpp>

// TODO: switch to rtmath throws
//       get rid of printfs
#include <cstdio>

#include "../rtmath/Public_Domain/bhmie.h"
/*
void main(void)
{
float x;
std::complex<double> cxref;
unsigned long nang;
std::complex<double> cxs1[mxnang], cxs2[mxnang];
float qext, qsca, qback, gsca;

void bhmie(float x, std::complex<double> cxref, unsigned long nang,
std::complex<double> cxs1[], std::complex<double> cxs2[],float  *qext,float  *qsca, float  *qback, 
float *gsca);

unsigned long i;
nang=10;
cxref=Complex(1.33,0.1);


x=0.1;
bhmie(x, cxref, nang, cxs1, cxs2, &qext, &qsca, &qback, &gsca);
printf(" x qext qsca gsca %f %f %f %f \n ", x,  qext, qsca, gsca);



for (i=1; i<=nang; i++) {
printf(" s1 r,i %i  %f %f %f %f \n ", i, cxs1[i].r, cxs1[i].i, 
cxs2[i].r, cxs2[i].i);
}

}
*/

namespace mie {
	namespace bhmie {

		void  bhmie(double x, const std::complex<double> &cxref, size_t nang, std::complex<double> cxs1[],
			std::complex<double> cxs2[], double &qext, double &qsca, double &qback, double &gsca)

			/*
			Subroutine BHMIE is the Bohren-Huffman Mie scattering subroutine
			to calculate scattering and absorption by a homogenous isotropic
			sphere.
			Given:
			X = 2*pi*a/lambda
			REFREL = (complex refr. index of sphere)/(real index of medium)
			NANG = number of angles between 0 and 90 degrees
			(will calculate 2*NANG-1 directions from 0 to 180 deg.)
			Returns:
			S1(1 .. 2*NANG-1) =  (incid. E perp. to scatt. plane,
			scatt. E perp. to scatt. plane)
			S2(1 .. 2*NANG-1) =  (incid. E parr. to scatt. plane,
			scatt. E parr. to scatt. plane)
			QEXT = C_ext/pi*a**2 = efficiency factor for extinction
			QSCA = C_sca/pi*a**2 = efficiency factor for scattering
			QBACK = 4*pi*(dC_sca/domega)/pi*a**2
			= backscattering efficiency
			GSCA = <cos(theta)> for scattering

			Original program taken from Bohren and Huffman (1983), Appendix A
			Modified by B.T.Draine, Princeton Univ. Obs., 90/10/26
			in order to compute <cos(theta)>
			This code was translatted to C by P. J. Flatau Feb 1998.

			*/

		{
			using namespace std;
			const size_t mxnang = 1000;
			const size_t nmxx = 3000;
			/* .. Array Arguments .. */
			/*      COMPLEX :: cxs1(2*mxnang-1), cxs2(2*mxnang-1)*/
			/* .. Local Scalars ..*/
			std::complex<double> cxan, cxan1, cxbn, cxbn1, cxxi, cxxi0, cxy, cxxi1;
			std::complex<double> cxtemp;
			double apsi, apsi0, apsi1, chi, chi0, chi1, dang, fn, p, pii, 
				rn, t, theta, xstop, ymod;
			double  dn, dx, psi, psi0, psi1;
			size_t jj, nmx, nn, nstop;
			/* .. Local Arrays ..*/
			std::complex<double> cxd[nmxx];
			double amu[mxnang], pi[mxnang], pi0[mxnang], pi1[mxnang], tau[mxnang];

			if (nang>mxnang){
				printf(" STOP '***Error: NANG > MXNANG in bhmie");
				return;
			}
			pii = boost::math::constants::pi<double>();
			dx = x;
			cxy = std::complex<double>(x,0.0)*cxref;

			/* Series expansion terminated after NSTOP terms */
			xstop = x + 4.0*pow(x,0.3333) + 2.0;
			nstop = (size_t) xstop;
			ymod = abs(cxy);
			nmx = (size_t) (std::max<double>(xstop,ymod) + 15.);

			if (nmx>nmxx) {
				printf(" x, nmx, nmxx, cxref %f %i %i  \n ", x, nmx, nmxx);
				printf(" xstop nstop ymod %f %i %f \n", xstop, nstop, ymod); 
				printf(" Error: NMX > NMXX= %i \n", nmxx);
				return;
			}


			dang = .5E0*pii/ (double)(nang-1);
			for (size_t j = 1; j<=nang; j++) {

				theta = (double)(j-1)*dang;
				amu[j] = cos(theta);
			}



			/* Logarithmic derivative D(J) calculated by downward recurrence
			beginning with initial value (0.,0.) at J=NMX */

			cxd[nmx] = complex<double>(0.E0,0.E0);
			nn = nmx - 1;

			for (size_t n = 1; n<= nn; n++) {
				rn = nmx - n + 1;
				/*        cxd(nmx-n) = (rn/cxy) - (1.E0/(cxd(nmx-n+1)+rn/cxy)) */
				cxtemp=cxd[nmx-n+1]+(complex<double>(rn,0.0)/cxy);
				cxtemp=1.0/cxtemp;
				cxd[nmx-n]=(complex<double>(rn,0.0)/cxy)-cxtemp;
			}

			for ( size_t j = 1; j <= nang; j++) {
				pi0[j] = 0.E0;
				pi1[j] = 1.E0;
			}
			nn = 2*nang - 1;
			for(size_t j = 1; j<= nn; j++) {
				cxs1[j] = complex<double>(0.E0,0.E0);
				cxs2[j] = complex<double>(0.E0,0.E0);
			}



			/* Riccati-Bessel functions with real argument X
			calculated by upward recurrence */

			psi0 = cos(dx);
			psi1 = sin(dx);
			chi0 = -sin(x);
			chi1 = cos(x);
			apsi0 = psi0;
			apsi1 = psi1;
			cxxi0 = std::complex<double>(apsi0,-chi0);
			cxxi1 = std::complex<double>(apsi1,-chi1);
			qsca = 0.E0;
			gsca = 0.E0;



			for ( size_t n = 1; n <= nstop; n++) {  

				dn = n;
				rn = n;
				fn = (2.E0*rn+1.E0)/(rn*(rn+1.E0));
				psi = (2.E0*dn-1.E0)*psi1/dx - psi0;
				apsi = psi;
				chi = (2.E0*rn-1.E0)*chi1/x - chi0;
				cxxi = std::complex<double>(apsi,-chi);
				/* Store previous values of AN and BN for use
				in computation of g=<cos(theta)> */
				if (n>1) {
					cxan1 = cxan;
					cxbn1 = cxbn;
				}



				/* Compute AN and BN:*/
				/*        cxan = (cxd(n)/cxref+rn/x)*apsi - apsi1; */

				cxan=cxd[n]/cxref;
				cxan+=complex<double>(rn/x,0.0);
				cxan*=apsi;
				cxan-=complex<double>(apsi1,0.0);


				/*        cxan = cxan/((cxd(n)/cxref+rn/x)*cxxi-cxxi1); */
				cxtemp=cxd[n]/cxref;
				cxtemp+=complex<double>(rn/x,0.0);
				cxtemp*=cxxi;
				cxtemp-=cxxi1;
				cxan/=cxtemp;

				/*        cxbn = (cxref*cxd(n)+rn/x)*apsi - apsi1; */
				cxbn=cxref*cxd[n];
				cxbn+=complex<double>(rn/x,0.0);
				cxbn*=apsi;
				cxbn-=complex<double>(apsi1,0.0);
				/*        cxbn = cxbn/((cxref*cxd(n)+rn/x)*cxxi-cxxi1); */
				cxtemp=cxref*cxd[n];
				cxtemp+=complex<double>(rn/x,0.0);
				cxtemp*=cxxi;
				cxtemp-=cxxi1;
				cxbn/=cxtemp;

				/* Augment sums for qsca and g=<cos(theta)> */
				/*        qsca = qsca + (2.*rn+1.)*(cabs(cxan)**2+cabs(cxbn)**2); */
				qsca += (2.*rn+1.)*(norm(cxan)+norm(cxbn)); 
				gsca += ((2.*rn+1.)/(rn*(rn+1.)))*norm(cxan); 

				if (n>1) {
					gsca += ((rn-1.)*(rn+1.)/rn)*(real(cxan1)*real(cxan)+
						imag(cxan1)*imag(cxan)+real(cxbn1)*real(cxbn)+imag(cxbn1)*imag(cxbn));
				}

				for ( size_t j = 1; j<= nang; j++) {
					jj = 2*nang - j;
					pi[j] = pi1[j];
					tau[j] = rn*amu[j]*pi[j] - (rn+1.E0)*pi0[j];
					p = pow(-1.0,(double)(n-1));
					/*          cxs1[j] = cxs1[j] + fn*(cxan*pi[j]+cxbn*tau[j]); */
					cxtemp=cxan*pi[j];
					cxtemp+=cxbn*tau[j];
					cxtemp*=fn;
					cxs1[j]+=cxtemp;
					t = pow(-1.0,(double)n);
					/*          cxs2[j] = cxs2[j] + fn*(cxan*tau[j]+cxbn*pi[j]); */
					cxtemp=cxan*tau[j];
					cxtemp+=cxbn*pi[j];
					cxtemp*=fn;
					cxs2[j]+=cxtemp;

					if (j!=jj) {
						/*            cxs1[jj] = cxs1[jj] + fn*(cxan*pi(j)*p+cxbn*tau(j)*t);*/
						cxtemp=cxan*pi[j]*p;
						cxtemp+=cxbn*tau[j]*t;
						cxtemp*=fn;
						cxs1[jj]+=cxtemp;

						/*            cxs2[jj] = cxs2[jj] + fn*(cxan*tau(j)*t+cxbn*pi(j)*p); */
						cxtemp=cxan*tau[j]*t;
						cxtemp+=cxbn*pi[j]*p;
						cxtemp*=fn;
						cxs2[jj]+=cxtemp;
					}
				}

				psi0 = psi1;
				psi1 = psi;
				apsi1 = psi1;
				chi0 = chi1;
				chi1 = chi;
				cxxi1 = std::complex<double>(apsi1,-chi1);

				/*  For each angle J, compute pi_n+1
				from PI = pi_n , PI0 = pi_n-1 */

				for ( size_t j = 1; j<= nang; j++) {
					pi1[j] = ((2.*rn+1.)*amu[j]*pi[j]-(rn+1.)*pi0[j])/rn;
					pi0[j] = pi[j];
				}
			} /*end of big for */




			/*  Have summed sufficient terms.
			Now compute qsca,qext,qback,and gsca */
			gsca *= 2. / qsca;
			qsca *= (2.E0/(x*x));
			qext = (4.E0/(x*x))*real(cxs1[1]);
			qback = (4.E0/(x*x))*norm(cxs1[2*nang-1]);


			return;
		}

	} ////
} ////
/*
Scalc::Scalc(const std::complex<double> &m, double x)
{
_m = m;
_x = x;
}

Scalc::~Scalc()
{
}

void Scalc::calc(double mu, double Snn[4][4], std::complex<double> Sn[4])
{
double qext, qsca, qback, gsca;
// bhmie isn't quite like my other method. I can enter the number of angles, 
// but I can't calculate for a specific angle.....
bhmie(_x, _m,
0, // number of angles
s1,
s2,
qext,
qsca,
qback,
gsca);
}


std::shared_ptr<rtmath::matrixop> miePhaseFunc::eval(double alpha) const
{
if (_eval_cache.count(alpha) > 0)
{
return _eval_cache[alpha];
}

double _mu = cos(alpha);

mie::Scalc sc(_m,_x);
double Snn[4][4];
std::complex<double> Sn[4];
sc.calc(_mu,Snn,Sn);
mie::Qcalc q(_m);
// For mie scattering, only P11, P12, P33 and P34 are unique
// All terms not based on them are zero
double Qext, Qabs, Qsca, g;
q.calc(_x,Qext,Qsca,Qabs,g);
rtmath::matrixop resa(2,4,4);

for (unsigned int i=0;i<4;i++)
for (unsigned int j=0;j<4;j++)
resa.set(4 * Snn[i][j] / (_x * _x * Qext), 2, i, j);

std::shared_ptr<rtmath::matrixop> res(new rtmath::matrixop(resa));
_eval_cache[alpha] = res;

return res;
}


} // end bhmie
} // end mie
*/