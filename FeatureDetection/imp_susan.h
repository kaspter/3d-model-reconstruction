#pragma once

namespace imp
{
	// =================================== SUSAN =====================================
	template<typename SourceValueType, class CastOpType> class SUSANImageFilter : public cv::BaseFilter
	{
		typedef typename CastOpType::type1 KernelValueType;
		typedef typename CastOpType::rtype ResultValueType;

		// Evaluation result cast operator
		CastOpType _castOp;

		// Filter direct parameters
		KernelValueType paramSigma;
		KernelValueType paramT;

		// Internal intermediate data structures
		std::vector<cv::Point>		 _pt;	// Kernel top-left based relative coordinates.
		std::vector<uchar*>			 _psrc; // Source image pixel data element pointer set.
		std::vector<KernelValueType> _kval; // Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
		//std::vector<KernelValueType> _ksum;	// Kernel values _ksum. Dynamic (as much elements as channels).
				
	public:
		SUSANImageFilter (unsigned radius, double sigma, double t, const CastOpType& castOp=CastOpType())
			: _castOp(castOp)
		{		
			ksize  = cv::Size(2*radius + 1, 2*radius + 1);
			anchor = cv::Point(radius, radius); 

			paramSigma = cv::saturate_cast<KernelValueType>(sigma);
			paramT	   = cv::saturate_cast<KernelValueType>(t);				

			cv::Mat matrix	= DiskMatrix_8uc1(radius); matrix.at<uchar>(anchor) = 0;
			for (int i = 0; i < matrix.rows; ++i)
			{
				int offset = i * matrix.step; // step equals width in this case
				for (int j = 0; j < matrix.cols; ++j)
				{
					if (matrix.data[j + offset] != 0) _pt.push_back(cv::Point(j,i));
				}
			}
			_psrc.resize(_pt.size());
		}

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{
			if(_kval.empty()) { _kval.resize(_pt.size()); }

			const SourceValueType** values		= (const SourceValueType**)&_psrc[0];
			const SourceValueType*  anchorValue = src[anchor.y] + anchor.x*cn;
			const cv::Point*		coords		= &_pt[0];

			KernelValueType*  coeffs = &_kval[0];
			
			KernelValueType totalKernelValue, convolutionValue, intensDiffRelation, sigmaSquareTwice = 2*paramSigma*paramSigma;
			for (int i, j, k, nElem = _pt.size(), nWidth = width * cn; dstcount > 0; --dstcount, ++src, dst += dststep)
		    {
				ResultValueType* output = (ResultValueType*)dst;

				for (k = 0; k < nElem; ++k)  
					values[k] = (const SourceValueType*)src[coords[k].y] + coords[k].x*cn;

				for (i = 0; i < nWidth; ++i)
				{
					totalKernelValue = 0;
					for (j = 0; j < nElem; ++j)
					{
						int dx = coords[j].x - anchor.x, dy = coords[j].y - anchor.y;
						intensDiffRelation = (values[j][i] - anchorValue[i]) / paramT;
						totalKernelValue += (coeffs[j] = std::exp(-((dx*dx + dy*dy) / sigmaSquareTwice) - (intensDiffRelation*intensDiffRelation)));
					}

					convolutionValue = 0;
					for (k = 0; k < nElem; ++k)
						convolutionValue += coeffs[k]*values[k][i];
					output[i] = _castOp(convolutionValue/totalKernelValue);
				}
			}
		}

		void reset() { _kval.clear(); }
	};

	class SUSANFeatureDetector
	{};	
}