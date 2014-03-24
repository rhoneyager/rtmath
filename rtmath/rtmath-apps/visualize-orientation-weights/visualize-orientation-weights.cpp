/* vtktest
* This program is designed to test computation and writing of hulls and delaunay
* triangulations for incorporation into the library core.
*/
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include <vtkVersion.h>
#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkScalarBarActor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>


#include <Ryan_Debug/debug.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-visualize-orientation-weights\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);
		//p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string >(), "input ddscat run")
			("betas,b", po::value<string >()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string >()->default_value("0:9:180:cos"), "Specify theta rotations")
			("phis,p", po::value<string >()->default_value("0:8:360:lin"), "Specify phi rotations")
			("mean_theta", po::value<double>()->default_value(0),
			"Theta mean (degrees)")
			("mean_phi", po::value<double>()->default_value(0),
			"Phi mean (degrees)")
			("kappa", po::value<double>()->default_value(10),
			"Kappa (degrees) is comparable to 1/sigma^2. Limit of infinity for uniform distribution, "
			"and 0 for only the mean.")
			("method", po::value<string>()->default_value("vMFdual"), "Specify the method used in the orientation "
			"calculations (vMF, vMFdual).");
		;

		hidden.add_options()
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		//if (vm.count("help") || argc == 1) doHelp("");

		std::string sbetas, sthetas, sphis;

		if (vm.count("betas"))
			sbetas = vm["betas"].as<std::string>();
		if (vm.count("thetas"))
			sthetas = vm["thetas"].as<std::string>();
		if (vm.count("phis"))
			sphis = vm["phis"].as<std::string>();

		rtmath::ddscat::rotations rot;
		// Also load in any ddscat.par files for recombination
		// interval is a dummy variable
		double bMin, bMax, tMin, tMax, pMin, pMax, interval;
		std::string specializer;
		size_t nB, nT, nP;
		rtmath::config::extractInterval(sbetas, bMin, bMax, interval, nB, specializer);
		//if (specializer != "lin")
		//	throw debug::xBadInput("Beta interval needs to be linearly spaced.");
		rtmath::config::extractInterval(sthetas, tMin, tMax, interval, nT, specializer);
		if (specializer != "cos")
			throw debug::xBadInput("Theta interval needs to be cosine spaced.");
		rtmath::config::extractInterval(sphis, pMin, pMax, interval, nP, specializer);
		if (specializer != "lin")
			throw debug::xBadInput("phi interval needs to be linearly spaced.");

		rot = rtmath::ddscat::rotations(bMin, bMax, nB, tMin, tMax, nT, pMin, pMax, nP);


		using namespace rtmath::ddscat::weights;
		ddWeightsDDSCAT dw(rot);

		double muT = vm["mean_theta"].as<double>();
		double muP = vm["mean_phi"].as<double>();
		double kappa = vm["kappa"].as<double>();

		std::string method;
		//method = "vmf";
		method = vm["method"].as<std::string>();
		std::transform(method.begin(), method.end(), method.begin(), ::tolower);

		boost::shared_ptr<OrientationWeights3d> ow, owiso;
		if (method == "vmf")
		{
			ow = boost::shared_ptr<OrientationWeights3d>(new VonMisesFisherWeights(
				dw, muT, muP, kappa));
			owiso = boost::shared_ptr<OrientationWeights3d>(new VonMisesFisherWeights(
				dw, muT, muP, 0));
		}
		else if (method == "vmfdual")
		{
			ow = boost::shared_ptr<OrientationWeights3d>(new BimodalVonMisesFisherWeights(
				dw, muT, muP, kappa));
			owiso = boost::shared_ptr<OrientationWeights3d>(new BimodalVonMisesFisherWeights(
				dw, muT, muP, 0.0001));
		}
		else doHelp("Unknown weighting method");


		// Create a sphere for some geometry
		vtkSmartPointer<vtkSphereSource> sphere =
			vtkSmartPointer<vtkSphereSource>::New();
		sphere->SetCenter(0, 0, 0);
		sphere->SetThetaResolution((int) nT);
		sphere->SetPhiResolution((int) nP);
		sphere->SetLatLongTessellation(1);
		sphere->SetRadius(1);
		sphere->Update();

		// Create scalar data to associate with the vertices of the sphere
		const double pi = boost::math::constants::pi<double>();
		int numPts = sphere->GetOutput()->GetPoints()->GetNumberOfPoints();
		vtkSmartPointer<vtkPoints> pts = sphere->GetOutput()->GetPoints();
		vtkSmartPointer<vtkFloatArray> scalars =
			vtkSmartPointer<vtkFloatArray>::New();
		scalars->SetNumberOfValues(numPts);
		double maxwt = 0;
		double minwt = 0;
		for (int i = 0; i < numPts; ++i)
		{
			Eigen::Vector3d x, angles;
			pts->GetPoint(i, x.data());
			double r = x.norm();
			angles(2) = acos(x(2)/r) * 180. / pi; // theta to ddscat, phi to vtk
			angles(0) = 1; // radius
			angles(1) = atan2(x(1),x(0)) * 180. / pi; // phi to ddscat, theta to vtk
			if (angles(1) < 0) angles(1) = 360. + angles(1);

			// Now that the angles are determined, look up the weight
			double weight = ow->getWeight(0, angles(2), angles(1));
			double weightiso = owiso->getWeight(0, angles(2), angles(1));

			weight /= weightiso;
			weight -= 1.;
			weight *= 100.;

			std::cerr << x.transpose() << "\t- " << angles.transpose() 
				<< "\t- " << weight << std::endl;
			if (weight > maxwt) maxwt = weight;
			if (weight < minwt) minwt = weight;

			scalars->SetValue(i, static_cast<float>(weight));
			//scalars->SetValue(i, static_cast<float>(i) / numPts);
		}
		vtkSmartPointer<vtkPolyData> poly =
			vtkSmartPointer<vtkPolyData>::New();
		poly->DeepCopy(sphere->GetOutput());
		poly->GetPointData()->SetScalars(scalars);

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
		mapper->SetInput(poly);
#else
		mapper->SetInputData(poly);
#endif
		mapper->ScalarVisibilityOn();
		mapper->SetScalarModeToUsePointData();
		mapper->SetColorModeToMapScalars();
		if (minwt < 0) minwt *= 1.2;
		else minwt /= 1.2;
		maxwt *= 1.20;
		mapper->SetScalarRange(minwt, maxwt);

		vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);

		vtkSmartPointer<vtkScalarBarActor> scalarBar =
			vtkSmartPointer<vtkScalarBarActor>::New();
		scalarBar->SetLookupTable(mapper->GetLookupTable());
		scalarBar->SetTitle("Wt. % Chg. Rel. to Iso.");
		scalarBar->SetNumberOfLabels(6);

		// Create a lookup table to share between the mapper and the scalarbar
		vtkSmartPointer<vtkLookupTable> hueLut =
			vtkSmartPointer<vtkLookupTable>::New();
		hueLut->SetTableRange(0, 1);
		hueLut->SetHueRange(0, 1);
		hueLut->SetSaturationRange(1, 1);
		hueLut->SetValueRange(1, 1);
		hueLut->Build();

		mapper->SetLookupTable(hueLut);
		scalarBar->SetLookupTable(hueLut);

		// Create a renderer and render window
		vtkSmartPointer<vtkRenderer> renderer =
			vtkSmartPointer<vtkRenderer>::New();

		renderer->GradientBackgroundOn();
		renderer->SetBackground(1, 1, 1);
		renderer->SetBackground2(0, 0, 0);

		vtkSmartPointer<vtkRenderWindow> renderWindow =
			vtkSmartPointer<vtkRenderWindow>::New();
		renderWindow->AddRenderer(renderer);

		// Create an interactor
		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
			vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);

		// Add the actors to the scene
		renderer->AddActor(actor);
		renderer->AddActor2D(scalarBar);

		// Render the scene (lights and cameras are created automatically)
		Ryan_Debug::waitOnExit(false);
		renderWindow->Render();
		renderWindowInteractor->Start();
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

