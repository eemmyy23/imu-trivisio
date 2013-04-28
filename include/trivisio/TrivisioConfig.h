#pragma once

#if defined(WIN32) && BUILD_SHARED_LIBS
#	ifdef Trivisio_EXPORTS
#		define Trivisio_DECLSPEC __declspec(dllexport)
#	else
#		define Trivisio_DECLSPEC __declspec(dllimport)
#	endif
#else
#	define Trivisio_DECLSPEC
#endif
