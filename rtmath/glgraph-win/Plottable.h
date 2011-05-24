#pragma once

namespace glgraphwin {

	public ref class Plottable abstract : public System::ComponentModel::Component
	{
	public:
		Plottable(void);
		virtual ~Plottable(void);
		virtual void Plot() = 0;
		property bool Visible
		{
			bool get() { return _visible; }
			void set(bool newvis) {_visible = newvis; }
		}
		//inline void Visible(bool newvis) {_visible = newvis;}
		//inline bool Visible() {return _visible;}
		void setAspectRatio(unsigned int width, unsigned int height);
	protected:
		bool _visible;
		unsigned int _controlWidth;
		unsigned int _controlHeight;
	};
	/*
	public ref class Color : public System::ComponentModel::Component
	{
	public:
		Color(double r, double g, double b, double a);
		Color();
		void Set(double r, double g, double b, double a);
		void Select();
	private:
		double _r, _g, _b, _a;
	};
	*/
	public ref class Shape abstract : public Plottable
	{
	public:
		Shape(double X, double Y, double Size, double Rotation);
		Shape();
		//virtual void Plot() = 0 override;
		// Colors are extracted as 32-bit integers
		System::Drawing::Color^ borderColor;
		System::Drawing::Color^ bodyColor;
	protected:
		double _x, _y, _size, _rotation;
	};

}; // end namespace
