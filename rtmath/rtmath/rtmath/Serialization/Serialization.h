#pragma once
/// Part of the serialization code, but this section only deals with 
/// handling file compression. Pulled from Ryan_Serialization to 
/// remove the rtmath dependency.

#include "../defs.h"

#pragma warning( push )
#pragma warning( disable : 4244 )
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#pragma warning( pop ) 

#include <ios>
#include <fstream>
#include <string>
#include <sstream>

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
}

namespace rtmath
{
	namespace serialization
	{
		/// \brief Disable auto-compression while saving. Files will still be compressed if given a recognized extension.
		void DLEXPORT_rtmath_core disable_auto_compression(bool);

		/// \brief Is auto-compression enabled or disabled?
		bool DLEXPORT_rtmath_core auto_compression_enabled();

		/** \brief Given a base name, see if there is a compressed file using this base (e.g. MyFile.txt -> MyFile.txt.bz2).
		* Return the compressed file path along with the compression method.
		*
		* \param base is the base name for the search.
		* \param meth is the recognized compression method extension. Empty if uncompressed.
		* \param target is the compressed file. Returns the original path if the initial file is found.
		* \param T is the type (std::string or boost::filesystem::path)
		* \returns Whether any file matching the base exists.
		**/
		template <typename T>
		bool DLEXPORT_rtmath_core detect_compressed(const T &base, std::string &meth, T &target);

		/// Convenience shortcut to compressed file detection.
		template <typename T>
		bool DLEXPORT_rtmath_core detect_compressed(const T &base);

		/** \brief Detects the type of compression, based on file extension, for a file.
		*
		* \param name is a file name (or path)
		* \param meth is the compression method extension. Blank if unrecognized / uncompressed.
		* \param T is the type (std::string or boost::filesystem::path)
		* \returns True / false depending on whether the compression type is recognized.
		* \note This function does not check for file existence.
		**/
		template <typename T>
		bool DLEXPORT_rtmath_core detect_compression(const T &name, std::string &meth);

		/** \brief Detect compression, and give equivalent uncompressed filename.
		*
		* \param meth is the compression method extension
		* \param uncompressed is the uncompressed filename
		* \param name is the raw file name
		* \param T is the type (std::string or boost::filesystem::path)
		* \returns True / false depending on whether the file is compressed with a recognized compression scheme.
		* \note This function does not check for file existence.
		**/
		template <typename T>
		bool DLEXPORT_rtmath_core uncompressed_name(const T &name,
			T &uncompressed,
			std::string &meth);

		/**
		* \brief Select compression based on file name. For saving.
		*
		* This function looks at a candidate filename and determines the compression method to use.
		* If the file has a recognized extension for compression, then this method is forced.
		* If a matching extension prohibiting compression is seen, then compression is inhibited (useful when
		* the serialization library is not being used for serialization).
		* Otherwise, selects the highest-priority compression algorithm.
		*
		* \param filename is the candidate file name.
		* \param meth is the extension of the selected compression method.
		* \param T is the type (std::string or boost::filesystem::path).
		* \returns True is compression was selected.
		* \returns False if no compression selected.
		**/
		template <typename T>
		bool DLEXPORT_rtmath_core select_compression(const T &filename, std::string &meth);

		/// \brief Defines the possible methods used for serialization.
		enum class serialization_method
		{
			XML,
			TEXT
		};

		/**
		* \brief Select the serialization method (xml or text) used in saving or loading.
		*
		* This function analyzes the candidate filename and determines the serialization
		* method to use. It can handle compressed files. If no extension matches the
		* internal lists, then the function uses the default value.
		*
		* \param T is the type (std::string or boost::filesystem::path).
		* \param filename is the candidate file name.
		* \param outname is the file name plus any indicating extension (if not found in the filename). Preserves compression extensions.
		* \see add_options
		* \see process_static_options
		**/
		template <typename T>
		serialization_method DLEXPORT_rtmath_core select_format(const T &filename, T &outname);

		template <typename T>
		serialization_method select_format(const T &filename)
		{
			T outname;
			return select_format(filename, outname);
		}

		/// Check to see if this is a serialization-known format. Works for either extensions of whole filenames.
		template <typename T>
		bool DLEXPORT_rtmath_core known_format(const T &filename);

		/// Check to see if this is a serialization-known format. Works for either extensions of whole filenames.
		template<typename T>
		bool DLEXPORT_rtmath_core known_format(const T &filename, const serialization_method &meth);

		/** \brief Get a list of extensions that serialization recognizes.
		*
		* Provides a list of recognized exentions, for plugins further on in a codebase.
		* \param includeCompressions adds in all extensions permuted with all compression methods
		* \param output is a comma-separated string listing all possible extensions.
		**/
		void DLEXPORT_rtmath_core known_formats(std::string &output, bool includeCompressions = false);

		/** \brief Get a list of compression formats that serialization recognizes.
		*
		* Provides a list of recognized exentions, for plugins further on in a codebase.
		* \param output is a comma-separated string listing all possible extensions.
		* \param base is an optional parameter providing a base extension (for convenience).
		**/
		void DLEXPORT_rtmath_core known_compressions(std::string &output, const std::string &base = "");


		// output and input streambufs supported
		/**
		* \brief Preps a boost::iostreams stream for decompression with the selected method.
		*
		* \param meth is the compressed filename extension.
		* \param sbuf is the boost::iostreams input stream
		**/
		void DLEXPORT_rtmath_core prep_decompression(const std::string &meth,
			boost::iostreams::filtering_istream &sbuf);

		/**
		* \brief Preps a boost::iostreams stream for compression with the selected method.
		*
		* \param meth is the compressed filename extension.
		* \param sbuf is the boost::iostreams output stream
		**/
		void DLEXPORT_rtmath_core prep_compression(const std::string &meth,
			boost::iostreams::filtering_ostream &sbuf);

		/**
		* \brief Adds passes shapestats options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		**/
		void DLEXPORT_rtmath_core add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/// Processes static options defined in add_options
		void DLEXPORT_rtmath_core process_static_options(
			boost::program_options::variables_map &vm);


	}

}
