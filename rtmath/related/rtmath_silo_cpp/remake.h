#pragma once

/// Template function for memory reallocation.
template <class T>
T *remake(T *ptr, int oldsize, int size)
{
	T *retval = new T[size];
	T *iptr = retval;
	for (int i = 0; i < oldsize; ++i)
		*iptr++ = ptr[i];
	delete[] ptr;
	return retval;
}
