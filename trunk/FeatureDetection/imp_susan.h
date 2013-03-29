#pragma once

namespace imp
{
	// =================================== SUSAN =====================================
	template<typename SourceValueType, class CastOpType> class SUSANImageFilter : public cv::BaseFilter
	{
		typedef typename CastOpType::type1 KernelValueType;
		typedef typename CastOpType::rtype ResultValueType;
		
		typedef double (__thiscall imp::SUSANImageFilter<SourceValueType, CastOpType>::*_fexp_ptr)(const cv::Point &, SourceValueType, SourceValueType);

		// Evaluation result cast operator
		CastOpType _castOp;

		// Filter direct parameters
		unsigned paramRadius;
		double	 paramSigma;
		double	 paramT;

		// Internal intermediate data structures
		std::vector<cv::Point>		 _pt;		// Kernel top-left based relative coordinates.
		std::vector<uchar*>			 _psrc;		// Source image pixel data element pointer set.
		std::vector<KernelValueType> _kval;		// Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
		cv::Mat						 _ctable;	// Filter exponent cache table. Is valuable for 8-bit integer SourceValueType only
		double						 _2sigma;	// 2nd power of paramSigma twice
		_fexp_ptr					 _fexp;		// Internal exponent-based equation computation function pointer, which depends on type size of image data


		double _fexp_u8(const cv::Point &at, SourceValueType nucleus, SourceValueType value)
		{
			int dx = at.x - anchor.x, dy = at.y - anchor.y;
			return _ctable.ptr<double>(abs(dx), abs(dy))[cv::saturate_cast<int>(value) - cv::saturate_cast<int>(nucleus) + (int)UCHAR_MAX];
		}

		double _fexp_xx(const cv::Point &at, SourceValueType nucleus, SourceValueType value)
		{
			int dx = at.x - anchor.x, dy = at.y - anchor.y;
			double tDiv = (value - nucleus) / paramT;
			return std::exp(-((dx*dx + dy*dy) / _2sigma) - (tDiv*tDiv));
		}
				
	public:
		SUSANImageFilter (unsigned radius, double sigma, double t, const CastOpType& castOp=CastOpType())
			: _castOp(castOp), paramRadius(0), paramSigma(1.0), paramT(1.0)
		{		
			init(radius, sigma, t);
		}

		void init(unsigned radius, double sigma = -1.0, double t = -1.0)
		{
			CV_Assert(radius > 0);

			if (sigma == -1.0) sigma = paramSigma;
			if (t     == -1.0) t     = paramT;

			CV_Assert(sigma > 0.0 && t > 0.0);

			if (radius != paramRadius)
			{
				paramRadius = radius;
				ksize		= cv::Size(2*radius + 1, 2*radius + 1);
				anchor		= cv::Point(radius, radius); 

				cv::Mat matrix = DiskMatrix_8uc1(radius); matrix.at<uchar>(anchor) = 0;
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

			if (sigma != paramSigma || t != paramT)
			{
				paramSigma  = sigma;
				paramT	    = t;	
				_2sigma		= 2 * sigma * sigma;

				_fexp = &imp::SUSANImageFilter<SourceValueType,CastOpType>::_fexp_xx;
				if (sizeof(SourceValueType) == 1)	//SourceValueType is UCHAR (u8 image)
				{
					int ctable_sizes[] = { anchor.x + 1, anchor.y + 1, 2 * UCHAR_MAX + 1 };
					_ctable.release(); _ctable.create(3, ctable_sizes, CV_64FC1);
					
					cv::Point *coords = &_pt[0];
					for (int p = 0, pmax = _pt.size(); p < pmax; ++p)
					{
						cv::Point pt = coords[p] - anchor;	
						if (pt.x < 0 || pt.y < 0) continue;

						double *cache_row = _ctable.ptr<double>(pt.x, pt.y);
						for (int i = 0; i < ctable_sizes[2]; ++i)
						{
							cache_row[i] = (this->*_fexp)(coords[p], ~(i - UCHAR_MAX) + (i <= UCHAR_MAX), i <= UCHAR_MAX ? 0 : UCHAR_MAX);
						}
					}
					_fexp = &imp::SUSANImageFilter<SourceValueType,CastOpType>::_fexp_u8;
				}
			}

			reset();
		}

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{
			if(_kval.empty()) { _kval.resize(_pt.size() * cn); }

			const SourceValueType** values = (const SourceValueType**)&_psrc[0];
			const cv::Point*		coords = &_pt[0];

			KernelValueType*		coeffs = &_kval[0];

			ResultValueType*		output;
			const SourceValueType*	anchorValue;
			
			KernelValueType totalKernelValue, convolutionValue;
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
						totalKernelValue += (coeffs[j] = cv::saturate_cast<KernelValueType>((this->*_fexp)(coords[j], anchorValue[i], values[j][i])));
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

	template<typename SourceValueType, class CastOpType> class SUSANResponseFilter : public cv::BaseFilter
	{
		typedef typename CastOpType::type1 KernelValueType;
		typedef typename CastOpType::rtype ResultValueType;
		
		typedef double (__thiscall imp::SUSANResponseFilter<SourceValueType, CastOpType>::*_fexp_ptr)(SourceValueType, SourceValueType);

		// Evaluation result cast operator
		CastOpType _castOp;

		// Filter direct parameters
		unsigned paramRadius;		
		double	 paramG;
		double	 paramT;		

		// Internal intermediate data structures
		std::vector<cv::Point>		 _pt;		// Kernel top-left based relative coordinates.
		std::vector<uchar*>			 _psrc;		// Source image pixel data element pointer set.
		std::vector<KernelValueType> _kval;		// Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
		std::vector<double>			 _ctable;	// Filter exponent cache table. Is valuable for 8-bit integer SourceValueType only
		_fexp_ptr					 _fexp;		// Internal exponent-based equation computation function pointer, which depends on type size of image data


		double _fexp_u8(SourceValueType nucleus, SourceValueType value)	{ return _ctable[cv::saturate_cast<int>(value) - cv::saturate_cast<int>(nucleus) + (int)UCHAR_MAX];	}
		double _fexp_xx(SourceValueType nucleus, SourceValueType value) { return std::exp(-pow((value - nucleus) / paramT,6)); }
				
	public:
		SUSANResponseFilter (unsigned radius, double g, double t, const CastOpType& castOp=CastOpType())
			: _castOp(castOp), paramRadius(0), paramG(1.0), paramT(1.0)
		{		
			init(radius, g, t);
		}

		void init(unsigned radius, double g = -1.0, double t = -1.0)
		{
			CV_Assert(radius > 0);

			if (g == -1.0) g = paramG;
			if (t == -1.0) t = paramT;

			CV_Assert(g > 0.0 && t > 0.0);

			if (radius != paramRadius)
			{
				paramRadius = radius;
				ksize		= cv::Size(2*radius + 1, 2*radius + 1);
				anchor		= cv::Point(radius, radius); 

				cv::Mat matrix = DiskMatrix_8uc1(radius);
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

			if (g != paramG || t != paramT)
			{
				paramG = g;
				paramT = t;	

				_fexp = &imp::SUSANResponseFilter<SourceValueType,CastOpType>::_fexp_xx;
				if (sizeof(SourceValueType) == 1)	//SourceValueType is UCHAR (u8 image)
				{
					_ctable.clear();
					_ctable.resize(2*UCHAR_MAX + 1);					

					double* values = &_ctable[0];					
					for (int i = 0, iMax = _ctable.size(); i < iMax; ++i)
						values[i] = (this->*_fexp)(~(i - UCHAR_MAX) + (i <= UCHAR_MAX), i <= UCHAR_MAX ? 0 : UCHAR_MAX);
					
					_fexp = &imp::SUSANResponseFilter<SourceValueType,CastOpType>::_fexp_u8;
				}
			}

			reset();
		}

		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
		{
			if(_kval.empty()) { _kval.resize(_pt.size() * cn); }

			const SourceValueType** values = (const SourceValueType**)&_psrc[0];
			const cv::Point*		coords = &_pt[0];

			ResultValueType*		output;
			const SourceValueType*	anchorValue;
			
			KernelValueType n, g = cv::saturate_cast<KernelValueType>(paramG);
			for (int i, j, k, nWidth = width * cn, nElem = _pt.size(); dstcount > 0; --dstcount, ++src, dst += dststep)
		    {
				output		= (ResultValueType*)dst;
				anchorValue = (const SourceValueType*)src[anchor.y] + anchor.x*cn;

				for (k = 0; k < nElem; ++k)  
					values[k] = (const SourceValueType*)src[coords[k].y] + coords[k].x*cn;

				for (i = 0; i < nWidth; ++i)
				{
					for (j = 0, n = 0; j < nElem; ++j)
						n += cv::saturate_cast<KernelValueType>((this->*_fexp)(anchorValue[i], values[j][i]));
					
					output[i] = _castOp(n < g ? g - n : KernelValueType(0));
				}
			}
		}

		void reset() { _kval.clear(); }
	};
}