#include "StdAfx.h"
#include "rtBaseAtmos.h"

namespace rtmathwinui {
	rtBaseAtmos::rtBaseAtmos(void)
	{
		InitializeComponent();
	}

	rtBaseAtmos::rtBaseAtmos(System::ComponentModel::IContainer ^container)
	{
		container->Add(this);
		InitializeComponent();
	}

	rtBaseAtmos::~rtBaseAtmos()
	{
		if (components)
		{
			delete components;
		}
	}


}; // end rtmathwinui


