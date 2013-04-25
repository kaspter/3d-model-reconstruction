
#include "precomp.h"
#include "imp_extension.h"

namespace imp
{

	cv::Mat diskMask(double radius)
	{
		assert(radius > 0);

		unsigned r  = static_cast<unsigned>(round(radius)), d = 2 * r + 1;
		double rpow = radius + 0.5; rpow *= rpow;
		
		cv::Mat res = cv::Mat(d, d, CV_8UC1);
		for(unsigned imax = d*d, i=0; i < imax ; ++i)
		{
			int x = i % d - r, y = i / d - r;
			res.at<uchar>(y + r,x + r) = (uchar)(static_cast<unsigned>(x*x + y*y) <= rpow);			
		}
		return res;
	}

	template<typename _ElemT>
	size_t nms_perform(const cv::Mat &src, cv::Mat &dst, _ElemT maxValue)
	{
		cv::Mat skip(cv::Size(src.cols, 2), CV_8UC1, cv::Scalar(0));
		uchar *skip_cur = skip.ptr<uchar>(0), 
			  *skip_nxt = skip.ptr<uchar>(1); 
		
		size_t maxima_count = 0;
		for (int r = 0, rmax = src.rows - 2; r < rmax; )
		{
			const _ElemT *buf_row  = src.ptr<_ElemT>(++r);

			for(int c = 0, cmax = src.cols - 2; c < cmax; )
			{
				if (skip_cur[++c]) continue;

				if (buf_row[c] <= buf_row[c + 1])
				{
					do { ++c; } while (c < cmax && buf_row[c] <= buf_row[c + 1]); 
					if (c == cmax) break;	
				}
				else if (buf_row[c] <= buf_row[c - 1]) continue;

				skip_cur[c + 1] = true;

				const _ElemT *buf_row_temp = src.ptr<_ElemT>(r + 1);
				if (buf_row[c] <= buf_row_temp[c - 1]) continue; /**/skip_nxt[c - 1] = true;
				if (buf_row[c] <= buf_row_temp[c	]) continue; /**/skip_nxt[c    ] = true;
				if (buf_row[c] <= buf_row_temp[c + 1]) continue; /**/skip_nxt[c + 1] = true;
				
				buf_row_temp = src.ptr<_ElemT>(r - 1);
				if (buf_row[c] <= buf_row_temp[c - 1]) continue;
				if (buf_row[c] <= buf_row_temp[c    ]) continue;
				if (buf_row[c] <= buf_row_temp[c + 1]) continue;

				dst.at<_ElemT>(r, c) = maxValue != _ElemT(0) ? maxValue : buf_row[c];
				++maxima_count;
			}

			swap(skip_cur, skip_nxt);
			memset(skip_nxt, 0, src.step);
		}
		return maxima_count;
	}
	size_t nonMaxSuppression3x3(const cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues)
	{
		CV_Assert(!src.empty() && isGraymap(src));

		cv::Mat buf;

		bool inPlace;
		if (inPlace = src.data == dst.data)
		{
			buf = src.clone();
		}
		else
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );

			buf = src;
			dst.create(buf.size(), buf.type());	
		}
		dst.setTo(cv::Scalar::all(0));
		
		int depth = src.depth();
		switch(depth)
		{
		case CV_8U:
			return nms_perform(buf, dst, preserveMaximaValues ? 
				std::numeric_limits<uchar>::max() : std::numeric_limits<uchar>::min());
			break;
		case CV_16U:
			return nms_perform(buf, dst, preserveMaximaValues ? 
				std::numeric_limits<ushort>::max() : std::numeric_limits<ushort>::min());
			break;
		case CV_32F:
			return nms_perform(buf, dst, float(0));
			break;
		}
		return 0;
	}

	template<typename _ElemT>
	void hiest_perform(const cv::Mat &src, cv::Mat &dst)
	{
		unsigned *hist = reinterpret_cast<unsigned*>(dst.data);
		for (int r = 0, rmax = src.rows; r < rmax; ++r)
		{
			const _ElemT *src_row = src.ptr<_ElemT>(r);
			for (int c = 0, cmax = src.cols; c < cmax; ++c)	
			{
				if (hist[src_row[c]] != std::numeric_limits<unsigned>::max()) ++(hist[src_row[c]]);
			}
		}
	}
	void discreteGraymapHistogram(const cv::Mat &src, cv::OutputArray &dst)
	{
		int sdepth = src.depth();
		CV_Assert( isGraymap(src) && sdepth != CV_32F );

		cv::Mat hist;
		switch (sdepth)
		{
		case CV_8U:
			dst.create(std::numeric_limits<uchar>::max() + 1, 1, cv::DataType<unsigned>::type);
			hiest_perform<uchar>(src, hist = dst.getMat());
			break;
		case CV_16U:
			dst.create(std::numeric_limits<ushort>::max() + 1, 1, cv::DataType<unsigned>::type);
			hiest_perform<uchar>(src, hist = dst.getMat());
			break;
		}
	}

} // namespace imp