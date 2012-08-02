#pragma once
/* image.h - Garrett single image processing class
 * This set of classes takes a single image and manipulates it 
 * as a PCL point cloud array. It is used for meshing, depth 
 * detection, and will be used by other classes to unify the 
 * three disparate images.
 *
 * This does not unify the images by itself, and it does not 
 * attempt image segmentation and registration. Those features 
 * belong in other classes.
 *
 * Since boost's graphics libs cannot read garrett's images by 
 * default, Magick++ is used as a loading backend for PCL
 */

#include <set>
#include <string>
#include <vector>

namespace rtmath
{
	class matrixop;

	namespace Garrett
	{
		class meshObj;

		class image
		{
			public:
				image();
				image(const std::string &filename);
				~image();
				void readPNG(const std::string &filename);
				void writePNG(const std::string &filename) const;
				void writePoints(const std::string &filename) const;
				void readPoints(const std::string &filename);
				void readMesh(const std::string &filename);
				void writeMesh(const std::string &filename) const;
			protected:
//				void _convertPCL();
				void _meshPCL();
				std::vector<matrixop> _points;
				meshObj _mesh;
		};
	}
}

