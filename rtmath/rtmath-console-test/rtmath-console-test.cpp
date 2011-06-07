#include <iostream>
#include "../rtmath/rtmath.h"
#include "../mie/mie.h"
#include "../rtmath-base/rtmath-base.h"
#include <complex>
#include <time.h>
//#include <netcdf.h>
#define _USE_MATH_DEFINES
#include <math.h>

void prnMat(rtmath::matrixop &mat)
{
	using namespace std;
	using namespace rtmath;
	vector<unsigned int> pos(2,0), resSize = mat.size();
	for (unsigned int i=0; i<resSize[0]; i++)
	{
		for (unsigned int j=0; j<mat.size()[1]; j++)
		{
			pos[0] = i;
			pos[1] = j;
			cout << mat.get(pos) << '\t';
		}
		cout << endl << endl;
	}
}

int main(int argc, char** argv)
{
	using namespace std;
	rtmath::debug::debug_preamble();
	cout << "Testing the basic mie library functions first\n";
	/*
	cout << "Testing wn for x = pi\n";
	mie::wnCalc wtest(3.14159);
	cout << "n\tW(1,n)\n";
	for (unsigned int i=0; i<10; i++)
	{
		complex<double> res = wtest.calc(i);
		cout << "w(" << i << ") = " << res << endl;
	}
	std::cin.get();
	cout << "Testing pi_n for theta = pi (mu = cos(pi))\n";
	mie::piNCalc pitest(3.14159);
	cout << "n\tpi(0,n)\n";
	for (unsigned int i=0; i<10; i++)
	{
		complex<double> res = pitest.calc(i);
		cout << "pi(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	cout << "Testing tau_n for theta = 0 (mu = 1)\n";
	mie::tauNCalc tautest(0.0);
	cout << "n\ttau(0,n)\n";
	for (unsigned int i=1; i<10; i++)
	{
		double res = tautest.calc(i);
		cout << "tau(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	cout << "Testing A_n for x = 1 (m = 1+0.33i)\n";
	mie::AnCalc Atest(2,complex<double>(1.1,0.63));
	cout << "n\tA(1,1+0.33i,n)\n";
	for (unsigned int i=0; i<16; i++)
	{
		complex<double> res = Atest.calc(i);
		cout << "A(" << i << ") = " << res << endl;
	}
	std::cin.get();
	
	std::complex<double> m(1.33,0);
	cout << "Testing the calculated an and bn\n";
	std::complex<double> an, bn;
	mie::abNCalc abncalc(m,2.0);
	for (unsigned int n=1;n<=10;n++)
	{
		abncalc.calc(n, an, bn);
		cout << n << "\t" << an << "\t" << bn << std::endl;
	}
	std::cin.get();
	cout << "Testing a few x points for the Qext code" << endl;

	mie::Qcalc q(m);
	for (double x=0.1;x<=1;x=x+0.1)
	{
		double Qe, Qa, Qs, g;
		q.calc(x,Qe,Qs,Qa, g);
		cout << Qe << std::endl;
	}
	
	std::cin.get();
	cout << "Testing the basic netCDF libraries\n";
	int fileid;
	int timeid;
	int voltageid;
	const char* desc = "The test attribute";
	nc_create("nctest.nc",0,&fileid);
	nc_def_dim(fileid, "time", NC_UNLIMITED, &timeid);
	nc_def_var(fileid, "voltage", NC_DOUBLE,1, &timeid, &voltageid);
	nc_put_att_text(fileid,voltageid,"desc",strlen(desc), desc);
	nc_enddef(fileid);
	double voltdata[] = {1.0, 2.0, 3.0, 3.5, 3.8, 2.5, 1.4, 1.2, 1.0, 0.5, 0.25};
	static size_t start[] = {0};
	static size_t count[] = {10};
	nc_put_vara_double(fileid, voltageid, start, count, voltdata);
	nc_close(fileid);
	cout << "netCDF file written!" << std::endl;
	std::cin.get();
	*/
	/*
	cout << "Gaussian quadrature test" << endl;
	cout << "Testing polynomial function y=4-x^2 with degrees 2-7" << endl;
	rtmath::polynomial x(1,1);
	rtmath::polynomial y = x*x * -1.0 + 4.0;
	for (unsigned int i=2; i<8; i++)
	{
		cout << i << "\t" << rtmath::quadrature::quad_eval_leg(-2.0,2.0,i,&y) << endl;
	}
	std::cin.get();
	
	cout << "Mie Mueller Matrix Calculation" << endl;
	std::complex<double> m(1.33,0);
	double x = 1.0;
	mie::Scalc Sc(m,x);
	double Snn[4][4];
	complex<double> Sn[4];
	Sc.calc(0,Snn,Sn);
	mie::miePhaseFunc mp;
	cout << "Sn:" << endl;
	for (unsigned int i=0; i<4;i++)
		cout << i << "\t" << Sn[i] << endl;
	cout << endl << "Snn:" << endl;
	for (unsigned int i=0;i<4;i++)
		for (unsigned int j=0;j<4;j++)
			cout << i << j << "\t" << Snn[i][j] << endl;
	double Pnn[4][4];
	mp.calc(0, m, x, Pnn);
	cout << endl << "Pnn:" << endl;
	for (unsigned int i=0;i<4;i++)
		for (unsigned int j=0;j<4;j++)
			cout << i << j << "\t" << Pnn[i][j] << endl;
	std::cin.get();
	*/
	cout << "Testing matrix math capabilities.\n";
	/*
	std::vector<unsigned int> szA, szB, szRes;
	// szA is 2x4, szB is 4x3
	szA.push_back(2);
	szA.push_back(4);
	szB.push_back(4);
	szB.push_back(3);
	szRes.push_back(2);
	szRes.push_back(3);
	rtmath::matrixop matA(szA);
	matA.set(1.0,2,0,0);
	matA.set(1.0,2,1,1);
	matA.set(2.0,2,0,1);
	matA.set(-3.0,2,1,0);
	prnMat(matA);
	cout << endl;
	rtmath::matrixop matB(szB);
	matB.set(1.0,2,0,0);
	matB.set(1.0,2,1,1);
	matB.set(5.0,2,1,0);
	matB.set(-7.0,2,0,1);
	prnMat(matB);
	cout << endl;
	rtmath::matrixop matRes(szRes);
	matRes = matA.operator*(matB);
	//matRes = matA * matB;
	cout << "Resulting matrix:\n";
	prnMat(matRes);
	*/
	/*
	rtmath::matrixop A(2,2,1), B(2,2,1), C(2,2,1), D(2,2,1);
	double Size = 1.0;
			A.set(-1.0/2.0,2,0,0); //     [-0.5, 0.5]
			A.set(1.0/2.0,2,1,0);
			B.set(Size/2.0,2,0,0); //     [ 0.5, 0.5]
			B.set(Size/2.0,2,1,0);
			C.set(Size/2.0,2,0,0); //     [ 0.5,-0.5]
			C.set(-1.0*Size/2.0,2,1,0);
			D.set(-1.0*Size/2.0,2,0,0);// [-0.5,-0.5]
			D.set(-1.0*Size/2.0,2,1,0);

			prnMat(A);
			prnMat(B);
			prnMat(C);
			prnMat(D);

			// Rotation is in radians
			//double Rotation = M_PI / 4.0;
			double Rotation = 0.0;
			rtmath::matrixop rot(2,2,2);
			rot.set(cos(Rotation),2,0,0);
			rot.set(cos(Rotation),2,1,1);
			rot.set(sin(Rotation),2,1,0);
			rot.set(-1.0*sin(Rotation),2,0,1);

			prnMat(rot);

			rtmath::matrixop Ar(2,2,1), Br(2,2,1), Cr(2,2,1), Dr(2,2,1);
			Ar = rot * A;
			Br = rot * B;
			Cr = rot * C;
			Dr = rot * D;

			prnMat(Ar);
			prnMat(Br);
			prnMat(Cr);
			prnMat(Dr);
			*/
	/*
	// Matrix inverse calculations
	rtmath::matrixop orig(2,3,3);
	orig.set(1,2,0,0);
	orig.set(2,2,0,1);
	orig.set(3,2,0,2);
	orig.set(4,2,1,0);
	orig.set(5,2,1,1);
	orig.set(6,2,1,2);
	orig.set(7,2,2,0);
	orig.set(8,2,2,1);
	orig.set(0,2,2,2);
	std::cout << "Inverse test: \nOriginal:\n";
	prnMat(orig);
	std::cout << orig.det() << std::endl;
	rtmath::matrixop min = orig.minor(2,0,0);
	std::cout << "Minor (0,0)\n";
	prnMat(min);
	std::cout << min.det() << std::endl;
	min = orig.minor(2,0,1);
	std::cout << "Minor (0,1)\n";
	prnMat(min);
	std::cout << min.det() << std::endl;
	min = orig.minor(2,1,0);
	std::cout << "Minor (1,0)\n";
	prnMat(min);
	std::cout << min.det() << std::endl;
	min = orig.minor(2,1,1);
	std::cout << "Minor (1,1)\n";
	prnMat(min);
	std::cout << min.det() << std::endl;
	// Invert
	std::cout << "Inverse: \n";
	rtmath::matrixop inv = orig.inverse();
	prnMat(inv);
	std::cout << inv.det() << std::endl;
	std::cin.get();
	*/
	// hitran loading tests
	using namespace std;
	cout << "hitran parse file: ";
	string htp, molp, pars;
	getline(cin,htp);
	cout << "Molparam file: ";
	getline(cin,molp);
	cout << "parsum file: ";
	getline(cin,pars);
	time_t start, finish;
	start = time (NULL);
	cout << "Start time: " << start << endl;
	rtmath::lbl::specline::loadlines("",molp.c_str(),pars.c_str());
	finish = time(NULL);
	cout << "Finish: " << finish << endl;
	cout << "Total time to load is " << (finish - start) << " seconds.\n";
	cout << "molp has " << rtmath::lbl::specline::abundanceMap.size() << " entries.\n";
	cout << "pars has " << rtmath::lbl::specline::Qmap.at(0).size() << " temperatures.\n";
	for (;;)
	{
		getline(cin,htp);
	}
	return 0;
}
