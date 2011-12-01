#pragma once
/* Contains common templates used in various functions.
 * Such as a null_deleter used to elude shared_ptr 
 * difficulties with *this.
 */

struct null_deleter
{
	template <class T> void operator()(T *) {}
};


