#pragma once
// daLayer - the fundamental class for constructing a scattering atmosphere
// Each layer is built up to a specified optical depth (tau) from an 
// infinitesimally-thin layer, which is added to itself, doubling in size 
// until the desured tau is reached.
// The initial layer-generating functions involved the doubling function.
// Prahl (1995) argues that this is better than successive scattering yet worse than 
// diamond initialization. I have no basis on which to verify this.
// Gaussian quaadrature is used because this is the most-frequently discussed method, 
// but Lobatto and Radau quadrature are also possible.

// The daLayer construction really depends on the phasefunction, tau, mu and mu_0,
// but mu and mu_0 are excluded from the constructor. Why? Because these only matter 
// when the entire layer is evaled.
// The exponentials that involve these are represented as diagonal matrices that 
// derive from damatrix and implement eval(). It is a better solution than specifying 
// mu and mu_0 at the beginning.

#include <memory>
#include "../matrixop.h"
#include "damatrix.h"
#include "daInitLayer.h"
#include "daPf.h"

namespace rtmath {
	class daLayer {
	public:
		daLayer(std::shared_ptr<damatrix> pf, double tau, double alb);
		virtual ~daLayer();
		virtual void generateLayer();							// Generates the layer if not yet generated
		std::shared_ptr<damatrix> R, T;							// The reflection and transmission matrices
		double tau();											// Get the current optical depth
		void tau(double newtau);								// Set the optical depth and regenerate the layer
	protected:
		std::shared_ptr<damatrix> _pf;							// The phase function matrix
		double _tau;											// Optical depth of layer
		double _alb;											// Single-scattering albedo of layer
		bool _layergenerated;									// Has the layer been validly generated
	};

}; // end namespace rtmath

