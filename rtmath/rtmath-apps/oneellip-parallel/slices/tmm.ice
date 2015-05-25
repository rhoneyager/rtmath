#pragma once
#include <IceStorm/IceStorm.ice>
//#include "Types.ice"

module rtmath
{
	module apps
	{
		module oneellipParallel
		{
			/**
			 * OneellipException contains all possible exceptions in the Oneellip subsystem.
			 **/
			exception OneellipException { string message; };

			enum messageId
			{
				NOOP,
				START,
				DONE,
				TERMINATE
			};

			enum eAeffVersion
			{
				VSPHERE,
				SASPHERE
			};

			enum eShapeType
			{
				SPHEROID,
				CYLINDER
			};

			dictionary<string, double> refrDic;

			struct inputParams
			{

				double beta = 0;
				double theta = 0;
				double phi = 0;
				double sTheta = 0;
				double sTheta0 = 0;
				double sPhi = 0;
				double sPhi0 = 0;

				double aeff = 0;
				eAeffVersion aeffVersion;
				double mRe = 0;
				double mIm = 0;
				double lambda = 0;
				string refrMeth;
				refrDic refrVals;

				bool aeffRescale = true;
				double vFrac = 0;
				string ref;

				eShapeType shapeType;

				double eps = 1;
			};

			sequence<inputParams> inputs;

			struct crossSections
			{
				string provider;
				double Qbk = 0;
				double Qext = 0;
				double Qsca = 0;
				double Qabs = 0;
				double g = 0;
				bool valid = false;

			};

			sequence<crossSections> outputs;

			struct message
			{
				messageId id;
				inputParams in;
				outputs cs;
			};

			sequence<message> messages;

			interface csProvider
			{
				["cpp:const"] idempotent messages doRun(inputs invals) throws OneellipException;
				["cpp:const"] idempotent int numProcessors();
			};

			interface csProviderFactory
			{
				csProvider* createCsProvider();
			};
		};
	};
	

};
