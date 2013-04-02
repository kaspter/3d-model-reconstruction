#pragma once // Despite the extension, this file, actually, plays a header role 
			 // because of templated classes member definition within.

namespace imp 
{
	// SUSAImageFilter implementation below

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	inline double SUSANImageFilter<SourceValueType, KernelValueType, ResultValueType>::_fexp_u8(const cv::Point &at, SourceValueType nucleus, SourceValueType value)
	{
		return _ctable.ptr<double>(abs(at.x), abs(at.y))[cv::saturate_cast<int>(value) - cv::saturate_cast<int>(nucleus) + (int)UCHAR_MAX];
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	inline double SUSANImageFilter<SourceValueType, KernelValueType, ResultValueType>::_fexp_xx(const cv::Point &at, SourceValueType nucleus, SourceValueType value)
	{
		double tDiv = (value - nucleus) / paramT;
		return std::exp(-((at.x*at.x + at.y*at.y) / _2sigma) - (tDiv*tDiv));
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	void SUSANImageFilter<SourceValueType, KernelValueType, ResultValueType>::init(unsigned radius, double sigma = -1.0, double t = -1.0)
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
					if (matrix.data[j + offset] != 0) _pt.push_back(cv::Point(j,i) - anchor);
				}
			}
			_psrc.resize(_pt.size());
		}

		if (sigma != paramSigma || t != paramT)
		{
			paramSigma  = sigma;
			paramT	    = t;	
			_2sigma		= 2 * sigma * sigma;

			_ctable.release();
			if (sizeof(SourceValueType) == 1)	//SourceValueType is UCHAR (u8 image)
			{
				int ctable_sizes[] = { anchor.x + 1, anchor.y + 1, 2 * UCHAR_MAX + 1 };
				_ctable.create(3, ctable_sizes, CV_64FC1);
					
				const cv::Point *coords = &_pt[0], *pt;
				for (int p = 0, pmax = _pt.size(); p < pmax; ++p)
				{
					pt = coords + p; if (pt->x < 0 || pt->y < 0) continue;

					double *cache_row = _ctable.ptr<double>(pt->x, pt->y);
					for (int i = 0; i < ctable_sizes[2]; ++i)
					{
						cache_row[i] = _fexp_xx(*pt, ~(i - UCHAR_MAX) + (i <= UCHAR_MAX), i <= UCHAR_MAX ? 0 : UCHAR_MAX);
					}
				}
				//_fexp = &SUSANImageFilter<SourceValueType, KernelValueType, ResultValueType>::_fexp_u8;
			}
		}

		reset();
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	void SUSANImageFilter<SourceValueType, KernelValueType, ResultValueType>::operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
	{
		if(_kval.empty()) { _kval.resize(_pt.size()); }

		const SourceValueType** values = (const SourceValueType**)&_psrc[0];
		const cv::Point*		coords = &_pt[0];
		KernelValueType*		coeffs = &_kval[0];
			
		bool precachedValue = !_ctable.empty();
		for (int i, j, k, nWidth = width * cn, nElem = _pt.size(); dstcount > 0; --dstcount, ++src, dst += dststep)
		{
			ResultValueType*		output		= (ResultValueType*)dst;
			const SourceValueType*	anchorValue = (const SourceValueType*)src[anchor.y] + anchor.x*cn;

			for (k = 0; k < nElem; ++k)
				values[k] = (const SourceValueType*)src[coords[k].y + anchor.y] + (coords[k].x + anchor.x)*cn;

			for (i = 0; i < nWidth; ++i)
			{
				KernelValueType totalKernelValue = 0;
				for (j = 0; j < nElem; ++j)
				{
					totalKernelValue += (coeffs[j] = cv::saturate_cast<KernelValueType>(
						precachedValue ? _fexp_u8(coords[j], anchorValue[i], values[j][i])
										: _fexp_xx(coords[j], anchorValue[i], values[j][i])
						));
				}
					
				KernelValueType convolutionValue = 0;
				for (k = 0; k < nElem; ++k)
					convolutionValue += coeffs[k]*values[k][i];
				output[i] = cv::saturate_cast<ResultValueType>(convolutionValue/totalKernelValue);
			}
		}
	}


	// SUSANFeatureDetector implementation below

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	inline double SUSANFeatureResponse<SourceValueType, KernelValueType, ResultValueType>::_fexp_u8(SourceValueType nucleus, SourceValueType value)
	{
		return _ctable[cv::saturate_cast<int>(value) - cv::saturate_cast<int>(nucleus) + (int)UCHAR_MAX];
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	inline double SUSANFeatureResponse<SourceValueType, KernelValueType, ResultValueType>::_fexp_xx(SourceValueType nucleus, SourceValueType value)
	{
		return std::exp(-pow((value - nucleus) / paramT,6));
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	void SUSANFeatureResponse<SourceValueType, KernelValueType, ResultValueType>::init(unsigned radius, double t = -1.0, double g = -1.0)
	{
		CV_Assert(radius > 0);

		if (t == -1.0) t = paramT;
		if (g == -1.0) g = paramG;

		CV_Assert(g > 0.0 && t > 0.0);

		if (radius != paramRadius)
		{
			paramRadius = radius;
			ksize		= cv::Size(2*radius + 1, 2*radius + 1);
			anchor		= cv::Point(radius, radius);

			_dist		= double(radius - 1);

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

			_ctable.clear();
			if (sizeof(SourceValueType) == 1)	//SourceValueType is UCHAR (u8 image)
			{
				_ctable.resize(2*UCHAR_MAX + 1);					

				double* values = &_ctable[0];					
				for (int i = 0, iMax = _ctable.size(); i < iMax; ++i)
					values[i] = _fexp_xx(~(i - UCHAR_MAX) + (i <= UCHAR_MAX), i <= UCHAR_MAX ? 0 : UCHAR_MAX);
			}
		}

		reset();
	}

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType>
	void SUSANFeatureResponse<SourceValueType, KernelValueType, ResultValueType>::operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn)
	{
		if(_kval.empty()) { _kval.resize(_pt.size()); }

		const SourceValueType** values = (const SourceValueType**)&_psrc[0];
		const cv::Point*		coords = &_pt[0];
		KernelValueType*		coeffs   = &_kval[0];
	
		bool precachedValue = !_ctable.empty(), isFeature;

		KernelValueType g = cv::saturate_cast<KernelValueType>(paramG);
		for (int i, j, k, nWidth = width * cn, nElem = _pt.size(); dstcount > 0; --dstcount, ++src, dst += dststep)
		{
			ResultValueType*		output		= (ResultValueType*)dst;
			const SourceValueType*	anchorValue = (const SourceValueType*)src[anchor.y] + anchor.x*cn;

			for (k = 0; k < nElem; ++k) 
			{
				values[k] = (const SourceValueType*)src[coords[k].y] + coords[k].x*cn;
			}

			for (i = 0; i < nWidth; ++i)
			{
				KernelValueType area	 = 0; 
				vec2d			centroid = vec2d();
				for (j = 0; j < nElem; ++j)
				{
					area += coeffs[j] = cv::saturate_cast<KernelValueType>(
						precachedValue ? _fexp_u8(anchorValue[i], values[j][i])
										: _fexp_xx(anchorValue[i], values[j][i])
						);
					centroid.x += coords[j].x * coeffs[j];
					centroid.y += coords[j].y * coeffs[j];
				}

				isFeature = false;
				if (area < g)
				{
					centroid.x /= area; centroid.x -= anchor.x;
					centroid.y /= area; centroid.y -= anchor.y;
					if ((centroid.x*centroid.x + centroid.y*centroid.y) >= _dist)
					{
						cv::Mat usan(ksize, CV_8UC1, cv::Scalar_<uchar>(0xFF));
						for (j = 0; j < nElem; ++j) 
						{
							if (std::ceil(coeffs[j] - 0.5) < 1.0) usan.at<uchar>(coords[j]) = 0x00;
						}

						isFeature = true;
						if (std::abs(centroid.x) >= std::abs(centroid.y)) 
						{	// by x
							int inc = sign(centroid.x), x = inc, xmax = inc * paramRadius; if (inc < 0) swap(x, xmax);
							for (; x <= xmax; ++x) 	
							{
								if (usan.at<uchar>(x + anchor.x, static_cast<int>(ROUND_VAL(centroid.y * x / centroid.x)) + anchor.y) == 0x00) { isFeature = false; break; }
							}
						}
						else
						{	// by y
							int inc = sign(centroid.y), y = inc, ymax = inc * paramRadius; if (inc < 0) swap(y, ymax);
							for (; y <= ymax; ++y) 	
							{
								if (usan.at<uchar>(static_cast<int>(ROUND_VAL(centroid.x * y / centroid.y)) + anchor.x, y + anchor.y) == 0x00) { isFeature = false; break; }
							}
						}
					}
				}

				output[i] = cv::saturate_cast<ResultValueType>(isFeature ? g - area : KernelValueType(0));
			}
		}
	}
}