#pragma once

namespace glgraphwin {

	class Plottable
	{
	public:
		Plottable(void);
		virtual ~Plottable(void);
		virtual void Plot() = 0;
		inline void Visible(bool newvis) {_visible = newvis;}
		inline bool Visible() {return _visible;}
	protected:
		bool _visible;
	};

	class Color
	{
	public:
		Color(double r, double g, double b, double a);
		Color();
		void Set(double r, double g, double b, double a);
		void Select();
	private:
		double _r, _g, _b, _a;
	};

	class Shape : public Plottable
	{
	public:
		Shape(double X, double Y, double Size, double Rotation);
		virtual void Plot() = 0;
		Color borderColor;
		Color bodyColor;
	protected:
		double _x, _y, _size, _rotation;
	};

}; // end namespace
