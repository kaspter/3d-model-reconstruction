#pragma once
#include "imp_extension.h"
#include <cmath>

namespace imp
{
//-----------------------------SUSAN-----------------------------
	template<typename ST, class CastOp> class SUSANImageFilter : public cv::BaseFilter
	{
		typedef typename CastOp::type1 KT;
		typedef typename CastOp::rtype DT;

		CastOp _castOp;

		KT paramSigma;
		KT paramT;

		std::vector<uchar>	diskMask;
		std::vector<cv::Point>	coords;
		std::vector<KT>		coeffs;
		std::vector<uchar*> ptrs;

		cv::Point anchorCoords;
		void generateKernel (const ST** kp, const ST* anchor, int cn)
		{			
			KT sigma2pow = 2*paramSigma*paramSigma;
			for( int i=0, imax = coeffs.size() - cn; i<imax;)
			{
				for (int k = i+cn; i<k; ++i )
				{					
					coeffs[i] = exp (
						-((coords[i].x*coords[i].x + coords[i].y*coords[i].y) / sigma2pow)
						-(kp[i] - anchor[k-i-1])/paramT)^2;
				}
			}
		}
		
	public:
		SUSANImageFilter (unsigned radius, double sigma, double t, const CastOp& castOp=CastOp())
			: _castOp(castOp), anchorCoords(radius+1, radius+1)
		{			
			paramSigma		= cv::saturate_cast<KT>(sigma);
			paramT			= cv::saturate_cast<KT>(t);				

			cv::Mat matrix	= DiskMatrix_8uc1(radius);
			matrix.at<uchar>( anchorCoords ) = 0;
			imp::preprocess2DKernel(matrix, coords, diskMask);

			ptrs.resize	( coords.size() );
		}

		void reset() { coeffs.clear(); }

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{
			const Point* pt = &coords[0];
			const KT*  kf	= (const KT*)&coeffs[0];
			const ST** kp	= (const ST**)&ptrs[0];
			
			if(coeffs.empty()) coeffs.resize ( coords.size()*cn );

			width *= cn;
	        for(; dstcount > 0; dstcount--, dst += dststep, src++ )
		    {
				DT* dstp = (DT*)dst;

			    for( k = 0; k < nonZero; k++ )  
					kp[k] = (const ST*)src[pt[k].y] + pt[k].x*cn;
				
				generateKernel (kp,(const ST*)src[anchorCoords.y] + anchorCoords.x*cn, cn);
				for( int i=0; i<width; i++ )
				{
					KT s0 = 0;
					for( k = 0; k < nonZero; k++ )
						s0 += kf[k]*kp[k][i];
					D[i] = _castOp(s0);
				}
			}

		}
	};

	class SUSANFeatureDetector
	{};	
}