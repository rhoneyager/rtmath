#pragma once

// From http://blogs.msdn.com/b/branbray/archive/2005/07/20/441099.aspx

// Third attempt at embedding a native class
// inside a ref class
template<typename T>
public ref class Embedded {
public:
	T* t;

	!Embedded() {
		if (t != nullptr) {
			delete t;
			t = nullptr;
		}
	}

	~Embedded() {
		this->!Embedded();
	}

public:
	Embedded() : t(new T) {}

	static T* operator&(Embedded% e) { return e.t; }
	static T* operator->(Embedded% e) { return e.t; }
};

/*
struct NativePoint {
	int x, y;
};


ref class R {
	Embedded<NativePoint> np;
};
*/
