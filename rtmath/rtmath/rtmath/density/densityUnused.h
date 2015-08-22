
		/// Convert between equivalent volumes for different substanses / phases
		/// Densities should be in g/cm^3.
		BOOST_PARAMETER_FUNCTION( (double),
			convertSubstanceDensity,
			tag,
			(optional
				(in_density, *, 0)
				(out_density, *, 0)
				(in_volume, *, 0)
				(in_aeff, *, 0)
				(temperature, *, 0)
				(temp_units, *, std::string("K"))
				(in_substance, *, std::string(""))
				(out_substance, *, std::string(""))
			) )
		{
				mylog( "convertSubstanceDensity\n"
					"\tin_density: " << in_density
				<< "\n\tout_density: " << out_density
				<< "\n\tin_volume: " << in_volume
				<< "\n\tin_aeff: " << in_aeff
				<< "\n\ttemp: " << temperature << " " << temp_units
				<< "\n\tin_subst: " << in_substance
				<< "\n\tout_subst: " << out_substance << std::endl);
			double inDen = in_density;
			double outDen = out_density;
			double inV = in_volume;
			double inAeff = in_aeff;
			bool doAeff = (inAeff > 0) ? true : false;
			if (!inV && !inAeff) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("Function requires either an input volume or "
						"effective radius to perform the conversion.");
			double outval = 0;
			if (in_substance == out_substance) {
				if (doAeff) return in_aeff;
				return in_volume;
			}
			// Suppressing the factors of pi, since this function always converts back to 
			// the same quantity specified in the input. As such, it only takes an effective 
			// radius of a volume. Max diameter or effective diameter won't work. Use convertLength.
			//const double pi = boost::math::constants::pi<double>();
			if (doAeff) inV = (4./3.) * pow(inAeff,3.);
			// Determine densities
			implementations::findDen(inDen, in_substance, temperature, temp_units);
			implementations::findDen(outDen, out_substance, temperature, temp_units);

			mylog("\n\tinDen " << inDen << "\toutDen: " << outDen << std::endl);
			// Once the densities are determined, then do the conversion of the volumes.
			// den = m / v.
			double mass = inDen * inV;
			outval = mass / outDen;

			if (doAeff) { // convert back to effective radius if requested
				outval = pow(3.*outval/(4.),1./3.);
			}
			mylog("\n\tresult is " << outval << " and doAeff is " << doAeff << std::endl);
			return outval;
		}

/// Function that calculates volume based on different aspect ratios. Assumes
		/// that the volume has the units of length^3.
		BOOST_PARAMETER_FUNCTION( (double),
			convertVolumeLength,
			tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string)) // can be Volume or a length that convertLength can handle
				(out_length_type, (std::string))
			)
			(optional
				(ar, *, 0.6)
				//(volume_fraction, *, 1)
			) )
		{
			mylog( "convertVolumeLength\n"
				<< "\tin_length_value: " << in_length_value
				<< "\n\tin_length_type: " << in_length_type
				<< "\n\tout_length_type: " << out_length_type
				<< "\n\tar: " << ar << std::endl);
			bool inIsV = false, outIsV = false;
			auto check = [](const std::string &name, const std::string &match, bool &out) {
				if (name == match) out = true; };
			check(in_length_type, "Volume", inIsV);
			check(out_length_type, "Volume", outIsV);
			// If input value is a volume, convert back to a length
			double inMD = 0;
			const double pi = boost::math::constants::pi<double>();
			if (inIsV) {
				if (ar <= 1) inMD = pow(6.*in_length_value*ar/pi,1./3.);
				else inMD = pow(6.*in_length_value/(pi*ar*ar),1./3.);
			} else {
				inMD = convertLength( _in_length_value = in_length_value,
				_in_length_type = in_length_type,
				_ar = ar,
				_out_length_type = "Max_Dimension"
				);
			}

			// inMD is the Max_Dimension
			double out = 0;
			if (outIsV) {
				if (ar <= 1) out = pow(inMD,3.) * pi / (6.*ar);
				else out = pow(inMD,3.) * pi * pow(ar,2.) / 6;
			} else {
				out = convertLength( _in_length_value = inMD,
					_in_length_type = "Max_Dimension",
					_ar = ar,
					_out_length_type = out_length_type
					);
			}
			mylog("\tresult is " << out);
			return out;
		}

