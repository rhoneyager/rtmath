#include "StdAfx.h"
#include "rtBaseAtmos.h"

namespace rtmathwinui {
	baseAtmos::baseAtmos(void)
	{
		InitializeComponent();
	}

	baseAtmos::baseAtmos(System::ComponentModel::IContainer ^container)
	{
		container->Add(this);
		InitializeComponent();
	}

	baseAtmos::~baseAtmos()
	{
		if (components)
		{
			delete components;
		}
	}


}; // end rtmathwinui


