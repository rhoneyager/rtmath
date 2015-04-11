/// \brief Provides ImageMagick file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <memory>

#include "plugin-ImageMagick.h"
#include <Magick++.h>


#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/images/image.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/io.h"
#include "../../rtmath/rtmath/plugin.h"

#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

namespace rtmath
{
	namespace plugins
	{
		namespace ImageMagick
		{
			boost::shared_ptr<Eigen::MatrixXf> readImage(const std::string& filename)
			{
				using namespace Magick;
				using namespace std;
				cerr << "\n" << filename << endl;

				// First, use ImageMagick to read the file
				Image img;
				try {
					img.read(filename);
				} catch( Magick::WarningCoder &warning ) {
					// Process coder warning while loading file (e.g. TIFF warning)
					// Maybe the user will be interested in these warnings (or not).
					// If a warning is produced while loading an image, the image
					// can normally still be used (but not if the warning was about
					// something important!)
					cerr << "Coder Warning: " << warning.what() << endl;
				} catch( Magick::Warning &warning ) {
					// Handle any other Magick++ warning.
					cerr << "Warning: " << warning.what() << endl;
				} catch( Magick::ErrorBlob &error ) { 
					// Process Magick++ file open error
					cerr << "Error: " << error.what() << endl;
					throw error;
				}

				/// \todo Can use a Magick function to do the reading. Switch.
				// Define the view area 
				int start_x = 0, start_y = 0;
				int size_x = (int) img.columns(), size_y = (int) img.rows();

				cout << "Image is " << size_x << " x " << size_y << endl;

				img.type(GrayscaleType);
				Pixels view(img);
				//PixelPacket* p = img.getPixels(0,0,img.columns(),img.rows());

				boost::shared_ptr<Eigen::MatrixXf> eim(new Eigen::MatrixXf(size_y, size_x));
				eim->setZero();

				for (size_t col=0; col< img.columns(); ++col)
					for (size_t row=0; row<img.rows(); ++row)
					{
						Quantum r = (view.get(col,row,1,1))->red;
						float zval = static_cast<float>(r)/static_cast<float>(QuantumRange);
						if (zval < 0.0005f) continue;

						(*eim)(row,col) = zval;
					}

					return eim;
			}

			void writeImage(const std::string &filename, 
				boost::shared_ptr<const Eigen::MatrixXf> out)
			{
				using namespace Magick;
				using namespace std;

				auto colorit = [](const float* in, size_t np, std::vector<float> &out)
				{
					out.resize(np*3);
					for (size_t i=0; i<np;++i)
					{
						for (size_t j=0;j<3;++j)
							out[3*i+j] = in[i];
					}
				};

				// First, use ImageMagick to read the PNG file
				std::vector<float> cdata;
				colorit(out->data(), out->rows() * out->cols(), cdata);
				Image img(out->rows(), out->cols(), "RGB", FloatPixel, cdata.data());
				img.write(filename);
			}
		}
	}

	namespace registry
	{
		template<>
		std::shared_ptr<IOhandler> 
			write_file_type_multi<rtmath::images::image>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::images::image > im)
		{
			using namespace plugins::ImageMagick;
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();
			std::string imkey = opts->getVal<std::string>("imkey", "raw");
			using std::shared_ptr;
			std::shared_ptr<ImageMagick_handle> h = registry::construct_handle
				<registry::IOhandler, ImageMagick_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<ImageMagick_handle>(
				new ImageMagick_handle(filename.c_str(), iotype)); });
			
			boost::shared_ptr<const Eigen::MatrixXf> eim;
			if (im->imageMaps.count(imkey)) eim = im->imageMaps.at(imkey);
			else RTthrow(debug::xMissingFile())
				<< debug::file_name(imkey);

			writeImage(filename, eim);

			return h; // Pass back the handle
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_multi<rtmath::images::image>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			boost::shared_ptr<rtmath::images::image > im,
			std::shared_ptr<const rtmath::registry::collectionTyped<rtmath::images::image> >)
		{
			using namespace plugins::ImageMagick;
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string imkey = opts->getVal<std::string>("imkey", "raw");
			using std::shared_ptr;
			std::shared_ptr<ImageMagick_handle> h = registry::construct_handle
				<registry::IOhandler, ImageMagick_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<ImageMagick_handle>(
				new ImageMagick_handle(filename.c_str(), iotype)); });


			boost::shared_ptr<const Eigen::MatrixXf> eim = readImage(filename);
			im->imageMaps[imkey] = eim;

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<rtmath::images::image>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::images::image> > &ims,
			std::shared_ptr<const rtmath::registry::collectionTyped<rtmath::images::image> >)
		{
			return sh;
		}
	}
}
