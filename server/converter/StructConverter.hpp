#ifndef _STRUCT_CONVERTER_HPP
#define _STRUCT_CONVERTER_HPP

#include <string.h>

namespace StructConverter {
	template<class T>
	void Struct2Buffer(const T* structPtr, char* buffer) {
		memcpy(buffer, structPtr, sizeof(T));
	}

	template<class T>
	void Buffer2Struct(const char* buffer, T* structPtr) {
		memcpy(structPtr, buffer, sizeof(T));
	}
}

#endif // !_STRUCT_CONVERTER_HPP
