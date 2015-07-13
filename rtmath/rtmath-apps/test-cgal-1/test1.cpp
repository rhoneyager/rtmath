// Program inputs shape points and uses them to make a mesh

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>

#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>
// IO
#include <CGAL/IO/Polyhedron_iostream.h>

#include <CGAL/Skin_surface_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/mesh_skin_surface_3.h>
#include <CGAL/subdivide_skin_surface_mesh_3.h>
#include "skin_surface_writer.h"
#include "write_c3t3_to_vtk_xml_file.h"
#include "write_c2t3_to_vtk_xml_file.h"
#include <list>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/error/debug.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Skin_surface_traits_3<K>                      Traits;
typedef CGAL::Skin_surface_3<Traits>                        Skin_surface_3;
typedef Skin_surface_3::FT                                  FT;
typedef Skin_surface_3::Weighted_point                      Weighted_point;
typedef Weighted_point::Point                               Bare_point;
typedef CGAL::Polyhedron_3<K,
  CGAL::Skin_surface_polyhedral_items_3<Skin_surface_3> >   Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

// Triangulation
#ifdef CGAL_CONCURRENT_MESH_3
  typedef CGAL::Mesh_triangulation_3<
    Mesh_domain,
    CGAL::Kernel_traits<Mesh_domain>::Kernel, // Same as sequential
    CGAL::Parallel_tag                        // Tag to activate parallelism
  >::type Tr;
#else
  typedef CGAL::Mesh_triangulation_3<Mesh_domain>::type Tr;
#endif

typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;
using namespace CGAL::parameters;

int main() {
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;
		cerr << "test1\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "Input files. File starts with "
			 "number of points, then is a list of x, y, z, weight.")

			("output,o", po::value<string>(), "Output base name")
			("shrinkfactor,f", po::value<float>()->default_value(0.7), "Shrink factor")
			;

		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &m)
		{
			std::cerr << desc << "\n" << m << std::endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		
		rtmath::debug::process_static_options(vm);

  std::list<Weighted_point> l;
  FT                        shrinkfactor = vm["shrinkfactor"].as<FT>(); //0.4;

  if (!vm.count("input")) doHelp("Need input file");
  if (!vm.count("output")) doHelp("Need output base");
  std::string inf = vm["input"].as<string>();
  std::string outbase = vm["output"].as<string>();

  std::ifstream iin(inf.c_str());
  size_t nPoints = 0;
  iin >> nPoints;
  for (size_t i=0; i<nPoints; ++i) {
	  float x, y, z, w;
	  iin >> x >> y >> z >> w;
	  l.push_front(Weighted_point(Bare_point(x,y,z), w);
  }
  //l.push_front(Weighted_point(Bare_point( 1,-1,-1), 1.25));
  //l.push_front(Weighted_point(Bare_point( 1, 1, 1), 1.25));
  //l.push_front(Weighted_point(Bare_point(-1, 1,-1), 1.25));
  //l.push_front(Weighted_point(Bare_point(-1,-1, 1), 1.25));
  //l.push_front(Weighted_point(Bare_point(-1,-1, 2), 1.10));
  //l.push_front(Weighted_point(Bare_point(-1,1, 2), 1.10));

  Polyhedron p;

  Skin_surface_3 skin_surface(l.begin(), l.end(), shrinkfactor);
  CGAL::mesh_skin_surface_3(skin_surface, p);

  CGAL::subdivide_skin_surface_mesh_3(skin_surface, p);

  string outmeshname = outbase;
  outmeshname.append("_mesh.off");
  std::ofstream out(outmeshname.c_str());
  out << p;

  using namespace CGAL;
  Mesh_domain domain(p);
  Mesh_criteria criteria(facet_angle=25, facet_size=0.15, facet_distance=0.008,
		  cell_radius_edge_ratio=3);
  C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
  std::string outmeditname = outbase;
  outmeditname.append("_medit.mesh");
  std::ofstream medit_file(outmeditname.c_str());
  c3t3.output_to_medit(medit_file);
  medit_file.close();
  std::string vtuname = outbase;
  vtuname.append("_mesh.vtu");
  CGAL::write_c3t3_to_vtk_xml_file(c3t3, vtuname);

  return 0;
}
