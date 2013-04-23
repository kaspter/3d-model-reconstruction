#pragma once

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