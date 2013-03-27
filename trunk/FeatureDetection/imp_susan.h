#pragma once

#include <fstream>

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
			const cv::Point*		coords		= &_pt[0];

			KernelValueType* coeffs = &_kval[0];

			ResultValueType*		output;
			const SourceValueType*	anchorValue;
			
								// std::ofstream fout("E:\\In-OUT\\CPP_Convolution.txt", std::ios::app); 
			KernelValueType totalKernelValue, convolutionValue, intensDiffRelation, sigmaSquareTwice = 2*paramSigma*paramSigma;
			for (int i, j, k, nWidth = width * cn, nElem = _pt.size(); dstcount > 0; --dstcount, ++src, dst += dststep)
		    {
				output		= (ResultValueType*)dst;
				anchorValue = (const SourceValueType*)src[anchor.y] + anchor.x*cn;

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
					
					//FILE fout = _topen(_T("E:\\In-OUT\\CPP_output2.txt"), _O_CREAT | _O_WRONLY | _O_TRUNC);

					convolutionValue = 0;
					for (k = 0; k < nElem; ++k)
						convolutionValue += coeffs[k]*values[k][i];
					output[i] = _castOp(convolutionValue/totalKernelValue);

								// if (convolutionValue > 0) fout << convolutionValue << "/" << totalKernelValue << "\n";
				}
			}

								// fout.close();
		}

		void reset() { _kval.clear(); }
	};

	class SUSANFeatureDetector
	{};	
}