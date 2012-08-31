#pragma once
/* Functions to calculate the density of ice 1-h and water (regular and supercooled)
 * for a given temperature. 
 */

namespace rtmath
{
	namespace density
	{
		// NOTE: these are all at standard pressure (1013.25 hPa)!!!!!

		/* For ice and supercooled water, 
		 * taken from the 2011-2012 (92nd) ed. of the Handbook of Chemistry and Physics
		 * http://www.hbcpnetbase.com/
		 * Saved as 06_44_91.pdf
 		*/

		/* Feistel, R ., and Wagner, W ., J. Phys. Chem. Ref. Data 35, 1021, 2006 .
		 * 3 .  International Association for the Properties of Water and Steam 
		 * (IAPWS), Revised Release on the Equation of State 2006 for H2
		 * O Ice Ih
		 * (2009), available from http://www .iapws .org
		 * */
		double ice1h(double Tk);

		//Wagner, W ., and Pruß A ., J. Phys. Chem. Ref. Data 31, 387, 2002
		double SuperWater(double Tk);


		double water(double Tk);
	}
}


