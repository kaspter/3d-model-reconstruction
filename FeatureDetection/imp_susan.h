#pragma once
#include "imp_extension.h"

namespace imp
{
//-------------------SUSAN-----------------------------
	template<typename ST, class CastOp, class VecOp> class SUSANImageFilter : public cv::BaseFilter
	{
		typedef typename CastOp::type1 KT;
		typedef typename CastOp::rtype DT;

		CastOp _castOp;
		VecOp  _vecOp;

		KT paramSigma;
		KT paramT;

	public:		
		SUSANImageFilter (double sigma, double t, const CastOp& castOp=CastOp(), const VecOp& vecOp=VecOp())
			: _castOp(castOp), _vecOp(vecOp)
		{
			paramSigma	= cv::saturate_cast<KT>(sigma);
			paramT		= cv::saturate_cast<KT>(t);
		}

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{			
		}
	};

	class SUSANFeatureDetector
	{};	
}