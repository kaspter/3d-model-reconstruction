#pragma once
#include "imp_extension.h"

namespace imp
{
//-------------------SUSAN-----------------------------
	template<typename ST, class CastOp, class VecOp> class SUSANImageFilter : public cv::BaseFilter
	{
		typedef typename CastOp::type1 KT;
		typedef typename CastOp::rtype DT;

	public:		
		SUSANImageFilter (KT sigma, KT t, const CastOp& _castOp=CastOp(), const VecOp& _vecOp=VecOp())
		{
		}

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{			
		}
	};

	class SUSANFeatureDetector
	{};	
}