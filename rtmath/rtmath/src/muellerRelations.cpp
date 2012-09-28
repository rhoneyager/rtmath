#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/muellerRelations.h"
#include "../rtmath/depGraph.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"

namespace rtmath
{
	namespace ddscat
	{
		namespace muellerRelations
		{
			muellerProvider::muellerProvider(const std::string &knownIndices)
				: recalcmaps(true)
			{
				constructGraph();
				if (knownIndices.size())
					addKnown(knownIndices);
				update();
			}

			bool muellerProvider::isKnown(const std::string& index) const
			{
				if (vertices.left.count(index) == 0) return false;
				auto obj = vertices.left.at(index);
				if (known.count(
					boost::weak_ptr<rtmath::graphs::vertex>(obj)))
					return true;
				return false;
			}

			bool muellerProvider::isUnknown(const std::string& index) const
			{
				if (vertices.left.count(index) == 0) return false;
				auto obj = vertices.left.at(index);
				if (unknown.count(
					boost::weak_ptr<rtmath::graphs::vertex>(obj)))
					return true;
				return false;
			}

			bool muellerProvider::isCalculable(const std::string& index) const
			{
				if (vertices.left.count(index) == 0) return false;
				auto obj = vertices.left.at(index);
				if (calculable.count(
					boost::weak_ptr<rtmath::graphs::vertex>(obj)))
					return true;
				return false;
			}

			void muellerProvider::addKnown(size_t index)
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			void muellerProvider::addKnown(const std::string &indices)
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			void muellerProvider::update()
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			void muellerProvider::constructGraph()
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			void muellerProvider::createVertex(const std::string &name, const std::string &target, 
				const std::string &deps, std::function<void(rtmath::matrixop&)> &)
			{
				throw rtmath::debug::xUnimplementedFunction();
			}

			matrixop muellerProvider::fillMask() const
			{
				throw rtmath::debug::xUnimplementedFunction();
				return matrixop(2,4,4);
			}

			matrixop muellerProvider::fill(const matrixop &in) const
			{
				throw rtmath::debug::xUnimplementedFunction();
				return matrixop(2,4,4);
			}

		}
	}
}

