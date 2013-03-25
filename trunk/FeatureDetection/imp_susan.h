#pragma once
#include "imp_extension.h"

namespace imp
{
//-----------------------------SUSAN-----------------------------
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
			const Point* pt = &coords[0];
			const KT* kf	= (const KT*)&coeffs[0];
			const ST** kp	= (const ST**)&ptrs[0];

			width *= cn;
	        for(; dstcount > 0; dstcount--, dst += dststep, src++ )
		    {
				DT* dstp = (DT*)dst;

			    for( k = 0; k < nz; k++ )  
					kp[k] = (const ST*)src[pt[k].y] + pt[k].x*cn;

				i = vecOp((const uchar**)kp, dst, width);

				for( ; i <= width - 4; i += 4 )
				{
					KT s0 = _delta, s1 = _delta, s2 = _delta, s3 = _delta;

					for( k = 0; k < nz; k++ )
					{
						const ST* sptr = kp[k] + i;
						KT f = kf[k];
						s0 += f*sptr[0];
						s1 += f*sptr[1];
						s2 += f*sptr[2];
						s3 += f*sptr[3];
					}

					dptr[i]		= _castOp(s0); 
					dptr[i+1]	= _castOp(s1);
					dptr[i+2]	= _castOp(s2); 
					dptr[i+3]	= _castOp(s3);
				}
			}

		}
	};

	class SUSANFeatureDetector
	{};	
}