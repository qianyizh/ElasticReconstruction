#pragma once

#include <pcl/registration/correspondence_rejection.h>
#include <pcl/point_cloud.h>

using namespace pcl;

template <typename SourceT, typename TargetT>
class PolyRejector: public registration::CorrespondenceRejector
{
	using registration::CorrespondenceRejector::input_correspondences_;
	using registration::CorrespondenceRejector::rejection_name_;
	using registration::CorrespondenceRejector::getClassName;

public:
	typedef boost::shared_ptr<PolyRejector> Ptr;
	typedef boost::shared_ptr<const PolyRejector> ConstPtr;

	typedef pcl::PointCloud<SourceT> PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

	typedef pcl::PointCloud<TargetT> PointCloudTarget;
	typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
	typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

	/** \brief Empty constructor */
	PolyRejector ()
		: iterations_ (10000)
		, cardinality_ (3)
		, similarity_threshold_ (0.75f)
		, similarity_threshold_squared_ (0.75f * 0.75f)
	{
		rejection_name_ = "PolyRejector";
	}

	/** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
	* \param[in] original_correspondences the set of initial correspondences given
	* \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
	*/
	void 
		getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
		pcl::Correspondences& remaining_correspondences);

	/** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
	* \param[in] cloud a cloud containing XYZ data
	*/
	inline void 
		setInputSource (const PointCloudSourceConstPtr &cloud)
	{
		input_ = cloud;
	}

	/** \brief Provide a source point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
	* \param[in] cloud a cloud containing XYZ data
	*/
	inline void 
		setInputCloud (const PointCloudSourceConstPtr &cloud)
	{
		PCL_WARN ("[pcl::registration::%s::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.\n",
			getClassName ().c_str ());
		input_ = cloud;
	}

	/** \brief Provide a target point cloud dataset (must contain XYZ data!), used to compute the correspondence distance.
	* \param[in] target a cloud containing XYZ data
	*/
	inline void 
		setInputTarget (const PointCloudTargetConstPtr &target)
	{
		target_ = target;
	}

	/** \brief See if this rejector requires source points */
	bool
		requiresSourcePoints () const
	{ return (true); }

	/*
	void
		setSourcePoints (pcl::PCLPointCloud2::ConstPtr cloud2)
	{ 
		PointCloudSourcePtr cloud (new PointCloudSource);
		fromPCLPointCloud2 (*cloud2, *cloud);
		setInputSource (cloud);
	}
	*/

	/** \brief See if this rejector requires a target cloud */
	bool
		requiresTargetPoints () const
	{ return (true); }

	/*
	void
		setTargetPoints (pcl::PCLPointCloud2::ConstPtr cloud2)
	{ 
		PointCloudTargetPtr cloud (new PointCloudTarget);
		fromPCLPointCloud2 (*cloud2, *cloud);
		setInputTarget (cloud);
	}
	*/

	/** \brief Set the polygon cardinality
	* \param cardinality polygon cardinality
	*/
	inline void 
		setCardinality (int cardinality)
	{
		cardinality_ = cardinality;
	}

	/** \brief Get the polygon cardinality
	* \return polygon cardinality
	*/
	inline int 
		getCardinality ()
	{
		return (cardinality_);
	}

	/** \brief Set the similarity threshold in [0,1[ between edge lengths,
	* where 1 is a perfect match
	* \param similarity_threshold similarity threshold
	*/
	inline void 
		setSimilarityThreshold (float similarity_threshold)
	{
		similarity_threshold_ = similarity_threshold;
		similarity_threshold_squared_ = similarity_threshold * similarity_threshold;
	}

	/** \brief Get the similarity threshold between edge lengths
	* \return similarity threshold
	*/
	inline float 
		getSimilarityThreshold ()
	{
		return (similarity_threshold_);
	}

	/** \brief Set the number of iterations
	* \param iterations number of iterations
	*/
	inline void 
		setIterations (int iterations)
	{
		iterations_ = iterations;
	}

	/** \brief Get the number of iterations
	* \return number of iterations
	*/
	inline int 
		getIterations ()
	{
		return (iterations_);
	}

	/** \brief Polygonal rejection of a single polygon, indexed by a subset of correspondences
	* \param corr all correspondences into \ref input_ and \ref target_
	* \param idx sampled indices into \b correspondences, must have a size equal to \ref cardinality_
	* \return true if all edge length ratios are larger than or equal to \ref similarity_threshold_
	*/
	inline bool 
		thresholdPolygon (const pcl::Correspondences& corr, const std::vector<int>& idx)
	{
		if (cardinality_ == 2) // Special case: when two points are considered, we only have one edge
		{
			return (thresholdEdgeLength (corr[ idx[0] ].index_query, corr[ idx[1] ].index_query,
				corr[ idx[0] ].index_match, corr[ idx[1] ].index_match,
				similarity_threshold_squared_));
		}
		else
		{ // Otherwise check all edges
			//for (int i = 0; i < cardinality_; ++i)
			//	if (!thresholdEdgeLength (corr[ idx[i] ].index_query, corr[ idx[(i+1)%cardinality_] ].index_query,
			//		corr[ idx[i] ].index_match, corr[ idx[(i+1)%cardinality_] ].index_match,
			//		similarity_threshold_squared_))
			//		return (false);
			for (int i = 0; i < cardinality_; ++i)
				for ( int j = i + 1; j < cardinality_; j++ )
					if (!thresholdEdgeLength (corr[ idx[i] ].index_query, corr[ idx[j] ].index_query,
						corr[ idx[i] ].index_match, corr[ idx[j] ].index_match,
						similarity_threshold_squared_))
						return (false);

			return (true);
		}
	}

	/** \brief Polygonal rejection of a single polygon, indexed by two point index vectors
	* \param source_indices indices of polygon points in \ref input_, must have a size equal to \ref cardinality_
	* \param target_indices corresponding indices of polygon points in \ref target_, must have a size equal to \ref cardinality_
	* \return true if all edge length ratios are larger than or equal to \ref similarity_threshold_
	*/
	inline bool 
		thresholdPolygon (const std::vector<int>& source_indices, const std::vector<int>& target_indices)
	{
		// Convert indices to correspondences and an index vector pointing to each element
		pcl::Correspondences corr (cardinality_);
		std::vector<int> idx (cardinality_);
		for (int i = 0; i < cardinality_; ++i)
		{
			corr[i].index_query = source_indices[i];
			corr[i].index_match = target_indices[i];
			idx[i] = i;
		}

		return (thresholdPolygon (corr, idx));
	}

protected:
	/** \brief Apply the rejection algorithm.
	* \param[out] correspondences the set of resultant correspondences.
	*/
	inline void 
		applyRejection (pcl::Correspondences &correspondences)
	{
		getRemainingCorrespondences (*input_correspondences_, correspondences);
	}

	/** \brief Get k unique random indices in range {0,...,n-1} (sampling without replacement)
	* \note No check is made to ensure that k <= n.
	* \param n upper index range, exclusive
	* \param k number of unique indices to sample
	* \return k unique random indices in range {0,...,n-1}
	*/
	inline std::vector<int> 
		getUniqueRandomIndices (int n, int k)
	{
		// Marked sampled indices and sample counter
		std::vector<bool> sampled (n, false);
		int samples = 0;
		// Resulting unique indices
		std::vector<int> result;
		result.reserve (k);
		do
		{
			// Pick a random index in the range
			const int idx = (std::rand () % n);
			// If unique
			if (!sampled[idx])
			{
				// Mark as sampled and increment result counter
				sampled[idx] = true;
				++samples;
				// Store
				result.push_back (idx);
			}
		}
		while (samples < k);

		return (result);
	}

	/** \brief Squared Euclidean distance between two points using the members x, y and z
	* \param p1 first point
	* \param p2 second point
	* \return squared Euclidean distance
	*/
	inline float 
		computeSquaredDistance (const SourceT& p1, const TargetT& p2)
	{
		const float dx = p2.x - p1.x;
		const float dy = p2.y - p1.y;
		const float dz = p2.z - p1.z;

		return (dx*dx + dy*dy + dz*dz);
	}

	/** \brief Edge length similarity thresholding
	* \param index_query_1 index of first source vertex
	* \param index_query_2 index of second source vertex
	* \param index_match_1 index of first target vertex
	* \param index_match_2 index of second target vertex
	* \param simsq squared similarity threshold in [0,1]
	* \return true if edge length ratio is larger than or equal to threshold
	*/
	inline bool 
		thresholdEdgeLength (int index_query_1,
		int index_query_2,
		int index_match_1,
		int index_match_2,
		float simsq)
	{
		// Distance between source points
		const float dist_src = computeSquaredDistance ((*input_)[index_query_1], (*input_)[index_query_2]);
		// Distance between target points
		const float dist_tgt = computeSquaredDistance ((*target_)[index_match_1], (*target_)[index_match_2]);
		// Edge length similarity [0,1] where 1 is a perfect match
		const float edge_sim = (dist_src < dist_tgt ? dist_src / dist_tgt : dist_tgt / dist_src);

		return (edge_sim >= simsq);
	}

	/** \brief Compute a linear histogram. This function is equivalent to the MATLAB function \b histc, with the
	* edges set as follows: <b> lower:(upper-lower)/bins:upper </b>
	* \param data input samples
	* \param lower lower bound of input samples
	* \param upper upper bound of input samples
	* \param bins number of bins in output
	* \return linear histogram
	*/
	std::vector<int> 
		computeHistogram (const std::vector<float>& data, float lower, float upper, int bins);

	/** \brief Find the optimal value for binary histogram thresholding using Otsu's method
	* \param histogram input histogram
	* \return threshold value according to Otsu's criterion
	*/
	int 
		findThresholdOtsu (const std::vector<int>& histogram);

	/** \brief The input point cloud dataset */
	PointCloudSourceConstPtr input_;

	/** \brief The input point cloud dataset target */
	PointCloudTargetConstPtr target_;

	/** \brief Number of iterations to run */
	int iterations_;

	/** \brief The polygon cardinality used during rejection */
	int cardinality_;

	/** \brief Lower edge length threshold in [0,1] used for verifying polygon similarities, where 1 is a perfect match */
	float similarity_threshold_;

	/** \brief Squared value if \ref similarity_threshold_, only for internal use */
	float similarity_threshold_squared_;
};

///////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> void 
	PolyRejector<SourceT, TargetT>::getRemainingCorrespondences (
	const pcl::Correspondences& original_correspondences, 
	pcl::Correspondences& remaining_correspondences)
{
	// This is reset after all the checks below
	remaining_correspondences = original_correspondences;

	// Check source/target
	if (!input_)
	{
		PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No source was input! Returning all input correspondences.\n",
			getClassName ().c_str ());
		return;
	}

	if (!target_)
	{
		PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No target was input! Returning all input correspondences.\n",
			getClassName ().c_str ());
		return;
	}

	// Check cardinality
	if (cardinality_ < 2)
	{
		PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Polygon cardinality too low!. Returning all input correspondences.\n",
			getClassName ().c_str() );
		return;
	}

	// Number of input correspondences
	const int nr_correspondences = static_cast<int> (original_correspondences.size ());

	// Not enough correspondences for polygonal rejections
	if (cardinality_ >= nr_correspondences)
	{
		PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Number of correspondences smaller than polygon cardinality! Returning all input correspondences.\n",
			getClassName ().c_str() );
		return;
	}

	// Check similarity
	if (similarity_threshold_ < 0.0f || similarity_threshold_ > 1.0f)
	{
		PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] Invalid edge length similarity - must be in [0,1]!. Returning all input correspondences.\n",
			getClassName ().c_str() );
		return;
	}

	// Similarity, squared
	similarity_threshold_squared_ = similarity_threshold_ * similarity_threshold_;

	// Initialization of result
	remaining_correspondences.clear ();
	remaining_correspondences.reserve (nr_correspondences);

	// Number of times a correspondence is sampled and number of times it was accepted
	std::vector<int> num_samples (nr_correspondences, 0);
	std::vector<int> num_accepted (nr_correspondences, 0);

	// Main loop
	for (int i = 0; i < iterations_; ++i)
	{
		// Sample cardinality_ correspondences without replacement
		const std::vector<int> idx = getUniqueRandomIndices (nr_correspondences, cardinality_);

		// Verify the polygon similarity
		if (thresholdPolygon (original_correspondences, idx))
		{
			// Increment sample counter and accept counter
			for (int j = 0; j < cardinality_; ++j)
			{
				++num_samples[ idx[j] ];
				++num_accepted[ idx[j] ];
			}
		}
		else
		{
			// Not accepted, only increment sample counter
			for (int j = 0; j < cardinality_; ++j)
				++num_samples[ idx[j] ];
		}
	}

	// Now calculate the acceptance rate of each correspondence
	std::vector<float> accept_rate (nr_correspondences, 0.0f);
	for (int i = 0; i < nr_correspondences; ++i)
	{
		const int numsi = num_samples[i];
		if (numsi == 0)
			accept_rate[i] = 0.0f;
		else
			accept_rate[i] = static_cast<float> (num_accepted[i]) / static_cast<float> (numsi);
	}

	// Compute a histogram in range [0,1] for acceptance rates
	const int hist_size = nr_correspondences / 2; // TODO: Optimize this
	const std::vector<int> histogram = computeHistogram (accept_rate, 0.0f, 1.0f, hist_size);

	// Find the cut point between outliers and inliers using Otsu's thresholding method
	const int cut_idx = findThresholdOtsu (histogram);
	const float cut = static_cast<float> (cut_idx) / static_cast<float> (hist_size);

	// Threshold
	for (int i = 0; i < nr_correspondences; ++i)
		if (accept_rate[i] > cut)
			remaining_correspondences.push_back (original_correspondences[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> std::vector<int> 
PolyRejector<SourceT, TargetT>::computeHistogram (const std::vector<float>& data,
	float lower, float upper, int bins)
{
	// Result
	std::vector<int> result (bins, 0);

	// Last index into result and increment factor from data value --> index
	const int last_idx = bins - 1;
	const float idx_per_val = static_cast<float> (bins) / (upper - lower);

	// Accumulate
	for (std::vector<float>::const_iterator it = data.begin (); it != data.end (); ++it)
		++result[ std::min (last_idx, int ((*it)*idx_per_val)) ];

	return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename SourceT, typename TargetT> int 
	PolyRejector<SourceT, TargetT>::findThresholdOtsu (const std::vector<int>& histogram)
{
	// Precision
	const double eps = std::numeric_limits<double>::epsilon();

	// Histogram dimension
	const int nbins = static_cast<int> (histogram.size ());

	// Mean and inverse of the number of data points
	double mean = 0.0;
	double sum_inv = 0.0;
	for (int i = 0; i < nbins; ++i)
	{
		mean += static_cast<double> (i * histogram[i]);
		sum_inv += static_cast<double> (histogram[i]);
	}
	sum_inv = 1.0/sum_inv;
	mean *= sum_inv;

	// Probability and mean of class 1 (data to the left of threshold)
	double class_mean1 = 0.0;
	double class_prob1 = 0.0;
	double class_prob2 = 1.0;

	// Maximized between class variance and associated bin value
	double between_class_variance_max = 0.0;
	int result = 0;

	// Loop over all bin values
	for (int i = 0; i < nbins; ++i)
	{
		class_mean1 *= class_prob1;

		// Probability of bin i
		const double prob_i = static_cast<double> (histogram[i]) * sum_inv;

		// Class probability 1: sum of probabilities from 0 to i
		class_prob1 += prob_i;

		// Class probability 2: sum of probabilities from i+1 to nbins-1
		class_prob2 -= prob_i;

		// Avoid division by zero below
		if (std::min (class_prob1,class_prob2) < eps || std::max (class_prob1,class_prob2) > 1.0-eps)
			continue;

		// Class mean 1: sum of probabilities from 0 to i, weighted by bin value
		class_mean1 = (class_mean1 + static_cast<double> (i) * prob_i) / class_prob1;

		// Class mean 2: sum of probabilities from i+1 to nbins-1, weighted by bin value
		const double class_mean2 = (mean - class_prob1*class_mean1) / class_prob2;

		// Between class variance
		const double between_class_variance = class_prob1 * class_prob2
			* (class_mean1 - class_mean2)
			* (class_mean1 - class_mean2);

		// If between class variance is maximized, update result
		if (between_class_variance > between_class_variance_max)
		{
			between_class_variance_max = between_class_variance;
			result = i;
		}
	}

	return (result);
}
