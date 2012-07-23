/*
 * Main DLL file.
 *
 * GDK: note to future fellow sufferers. If the Biclops service crashes with a "module not found"
 * error, and you have the BiclopsAccessManaged.dll (or whatever) in the right place, you should
 * also check for the location of pthreadVSE.dll. It needs to be copied into the MRDS bin file.
 * Otherwise bad things happen for no reason.
 *
 */

#include "BiclopsAccessManaged.h"

using namespace System;

#pragma managed


namespace BiclopsAccessManaged
{

	BiclopsAccessWrapper::BiclopsAccessWrapper() 
	{
		printf("getting Biclops Managed Wrapper\n");
		_biclops = BiclopsAccess::instance();
	}
}