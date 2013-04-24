#pragma once

template <class _Derived, class _Base>
_Derived *virtual_cast(_Base *ptr)
{
#if __RTTI
	return dynamic_cast<_Derived*>(ptr);
#else
	static volatile bool _is_init = false;
	static uintptr_t	 _offset  = 0;
	if (!_is_init)
	{
		_Derived _instance;
		_offset = reinterpret_cast<uintptr_t>(&_instance)
			- reinterpret_cast<uintptr_t>(static_cast<_Base*>(&_instance));

		_is_init = true;
	}

	return ptr == NULL ? NULL 
		: reinterpret_cast<_Derived*>(reinterpret_cast<uintptr_t>(ptr) + _offset);
#endif
}

template <class _Derived, class _Base>
struct _CallProxy 
{
	static _Derived &ref(_Base *ptr)
	{
		_Derived *_dptr = virtual_cast<_Derived, _Base>(ptr); 
		assert(_dptr != NULL);
		return *_dptr;
	}
};

#define IMP_MANAGER_CLASS(classname, baseclass, declarations)	\
class classname##Manager : public baseclass					\
{																\
	typedef _CallProxy<classname, baseclass> MyProxy;			\
public:															\
	declarations;												\
};

#define IMP_INIT_ALGORITHM(classname, algname, memberinit)															\
    static ::cv::Algorithm* create##classname##_hidden()															\
    {																												\
        return new classname;																						\
    }																												\
																													\
    static ::cv::AlgorithmInfo& classname##_info_auto = ::cv::AlgorithmInfo(algname, create##classname##_hidden);	\
																													\
    ::cv::AlgorithmInfo* classname::info() const																	\
    {																												\
        static volatile bool initialized = false;																	\
																													\
        if( !initialized )																							\
        {																											\
            initialized = true;																						\
			classname obj;																							\
            memberinit;																								\
        }																											\
        return &classname##_info_auto;																				\
    }


