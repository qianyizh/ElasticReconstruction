#pragma once

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_validation.h>
#include "PolyRejector.h"

using namespace pcl;

template <typename PointSource, typename PointTarget, typename FeatureT>
class RansacCurvature : public Registration<PointSource, PointTarget>
{
public:
	using Registration<PointSource, PointTarget>::reg_name_;
	using Registration<PointSource, PointTarget>::getClassName;
	using Registration<PointSource, PointTarget>::input_;
	using Registration<PointSource, PointTarget>::target_;
	using Registration<PointSource, PointTarget>::tree_;
	using Registration<PointSource, PointTarget>::max_iterations_;
	using Registration<PointSource, PointTarget>::corr_dist_threshold_;
	using Registration<PointSource, PointTarget>::transformation_;
	using Registration<PointSource, PointTarget>::final_transformation_;
	using Registration<PointSource, PointTarget>::transformation_estimation_;
	using Registration<PointSource, PointTarget>::getFitnessScore;
	using Registration<PointSource, PointTarget>::converged_;

	Eigen::Matrix< double, 6, 6 > information_source_;
	Eigen::Matrix< double, 6, 6 > information_target_;

	typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

	typedef typename Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;

	typedef PointIndices::Ptr PointIndicesPtr;
	typedef PointIndices::ConstPtr PointIndicesConstPtr;

	typedef pcl::PointCloud<FeatureT> FeatureCloud;
	typedef typename FeatureCloud::Ptr FeatureCloudPtr;
	typedef typename FeatureCloud::ConstPtr FeatureCloudConstPtr;

	typedef boost::shared_ptr<RansacCurvature<PointSource, PointTarget, FeatureT> > Ptr;
	typedef boost::shared_ptr<const RansacCurvature<PointSource, PointTarget, FeatureT> > ConstPtr;

	typedef typename KdTreeFLANN<FeatureT>::Ptr FeatureKdTreePtr;

	typedef PolyRejector<PointSource, PointTarget> CorrespondenceRejectorPoly;
	typedef typename CorrespondenceRejectorPoly::Ptr CorrespondenceRejectorPolyPtr;
	typedef typename CorrespondenceRejectorPoly::ConstPtr CorrespondenceRejectorPolyConstPtr;

	/** \brief Constructor */
	RansacCurvature ()
		: input_features_ ()
		, target_features_ ()
		, nr_samples_(3)
		, k_correspondences_ (2)
		, feature_tree_ (new pcl::KdTreeFLANN<FeatureT>)
		, correspondence_rejector_poly_ (new CorrespondenceRejectorPoly)
		, inlier_fraction_ (0.0f)
		, inlier_number_ ( 1000000 )
		, angle_diff_ (M_PI)
	{
		reg_name_ = "RansacCurvature";
		correspondence_rejector_poly_->setSimilarityThreshold (0.6f);
		max_iterations_ = 5000;
		transformation_estimation_.reset (new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget>);
	};

	/** \brief Destructor */
	virtual ~RansacCurvature ()
	{
	}

	void writeAuxData( const std::string & filename );

	/** \brief Provide a boost shared pointer to the source point cloud's feature descriptors
	* \param features the source point cloud's features
	*/
	void 
		setSourceFeatures (const FeatureCloudConstPtr &features);

	/** \brief Get a pointer to the source point cloud's features */
	inline const FeatureCloudConstPtr
		getSourceFeatures () const
	{ 
		return (input_features_);
	}

	/** \brief Provide a boost shared pointer to the target point cloud's feature descriptors
	* \param features the target point cloud's features
	*/
	void 
		setTargetFeatures (const FeatureCloudConstPtr &features);

	/** \brief Get a pointer to the target point cloud's features */
	inline const FeatureCloudConstPtr 
		getTargetFeatures () const
	{
		return (target_features_);
	}

	/** \brief Set the number of samples to use during each iteration
	* \param nr_samples the number of samples to use during each iteration
	*/
	inline void 
		setNumberOfSamples (int nr_samples)
	{
		nr_samples_ = nr_samples;
	}

	/** \brief Get the number of samples to use during each iteration, as set by the user */
	inline int 
		getNumberOfSamples () const
	{
		return (nr_samples_);
	}

	/** \brief Set the number of neighbors to use when selecting a random feature correspondence.  A higher value will
	* add more randomness to the feature matching.
	* \param k the number of neighbors to use when selecting a random feature correspondence.
	*/
	inline void
		setCorrespondenceRandomness (int k)
	{
		k_correspondences_ = k;
	}

	/** \brief Get the number of neighbors used when selecting a random feature correspondence, as set by the user */
	inline int
		getCorrespondenceRandomness () const
	{
		return (k_correspondences_);
	}

	/** \brief Set the similarity threshold in [0,1[ between edge lengths of the underlying polygonal correspondence rejector object,
	* where 1 is a perfect match
	* \param similarity_threshold edge length similarity threshold
	*/
	inline void
		setSimilarityThreshold (float similarity_threshold)
	{
		correspondence_rejector_poly_->setSimilarityThreshold (similarity_threshold);
	}

	/** \brief Get the similarity threshold between edge lengths of the underlying polygonal correspondence rejector object,
	* \return edge length similarity threshold
	*/
	inline float
		getSimilarityThreshold () const
	{
		return correspondence_rejector_poly_->getSimilarityThreshold ();
	}

	/** \brief Set the required inlier fraction (of the input)
	* \param inlier_fraction required inlier fraction, must be in [0,1]
	*/
	inline void
		setInlierFraction (float inlier_fraction)
	{
		inlier_fraction_ = inlier_fraction;
	}

	inline void
		setInlierNumber( int inlier_number )
	{
		inlier_number_ = inlier_number;
	}

	/** \brief Get the required inlier fraction
	* \return required inlier fraction in [0,1]
	*/
	inline float
		getInlierFraction () const
	{
		return inlier_fraction_;
	}

	/** \brief Get the inlier indices of the source point cloud under the final transformation
	* @return inlier indices
	*/
	inline const std::vector<int>&
		getInliers () const
	{
		return inliers_;
	}

	inline void setAngleDiff( float angle_diff ) {
		angle_diff_ = angle_diff;
	}

	inline bool thresholdNormal( const std::vector<int>& source_indices, const std::vector<int>& target_indices, const Eigen::Matrix4f & transformation ) {
		for ( int i = 0; i < ( int )source_indices.size(); i++ ) {
			Eigen::Vector4f ns( ( *input_ )[ source_indices[ i ] ].normal_x, ( *input_ )[ source_indices[ i ] ].normal_y, ( *input_ )[ source_indices[ i ] ].normal_z, 0 );
			Eigen::Vector4f nt( ( *target_ )[ target_indices[ i ] ].normal_x, ( *target_ )[ target_indices[ i ] ].normal_y, ( *target_ )[ target_indices[ i ] ].normal_z, 0 );
			Eigen::Vector4f nn = transformation * ns;
			if ( nt.dot( nn ) < cos( angle_diff_ ) ) {
				return false;
			}
		}
		return true;
	}

	void getInformation();

	void align_redux( PointCloudSource &output, const Eigen::Matrix4f& guess );

	void getDenseCorrespondences( std::vector< int > & corres, std::vector< float > & dis2 );

protected:
	/** \brief Choose a random index between 0 and n-1
	* \param n the number of possible indices to choose from
	*/
	inline int 
		getRandomIndex (int n) const
	{
		return (static_cast<int> (n * (rand () / (RAND_MAX + 1.0))));
	};

	/** \brief Select \a nr_samples sample points from cloud while making sure that their pairwise distances are 
	* greater than a user-defined minimum distance, \a min_sample_distance.
	* \param cloud the input point cloud
	* \param nr_samples the number of samples to select
	* \param sample_indices the resulting sample indices
	*/
	void 
		selectSamples (const PointCloudSource &cloud, int nr_samples, std::vector<int> &sample_indices);

	/** \brief For each of the sample points, find a list of points in the target cloud whose features are similar to 
	* the sample points' features. From these, select one randomly which will be considered that sample point's 
	* correspondence.
	* \param sample_indices the indices of each sample point
	* \param similar_features correspondence cache, which is used to read/write already computed correspondences
	* \param corresponding_indices the resulting indices of each sample's corresponding point in the target cloud
	*/
	void 
		findSimilarFeatures (const std::vector<int> &sample_indices,
		std::vector<std::vector<int> >& similar_features,
		std::vector<int> &corresponding_indices);

	/** \brief Rigid transformation computation method.
	* \param output the transformed input point cloud dataset using the rigid transformation found
	* \param guess The computed transformation
	*/
	void 
		computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess);

	/** \brief Obtain the fitness of a transformation
	* The following metrics are calculated, based on
	* \b final_transformation_ and \b corr_dist_threshold_:
	*   - Inliers: the number of transformed points which are closer than threshold to NN
	*   - Error score: the MSE of the inliers  
	* \param inliers indices of source point cloud inliers
	* \param fitness_score output fitness score as RMSE 
	*/
	void 
		getFitness (std::vector<int>& inliers, std::vector<int> & inliers_target, float& fitness_score);

	/** \brief The source point cloud's feature descriptors. */
	FeatureCloudConstPtr input_features_;

	/** \brief The target point cloud's feature descriptors. */
	FeatureCloudConstPtr target_features_;  

	/** \brief The number of samples to use during each iteration. */
	int nr_samples_;

	/** \brief The number of neighbors to use when selecting a random feature correspondence. */
	int k_correspondences_;

	/** \brief The KdTree used to compare feature descriptors. */
	FeatureKdTreePtr feature_tree_;

	/** \brief The polygonal correspondence rejector used for prerejection */
	CorrespondenceRejectorPolyPtr correspondence_rejector_poly_;

	/** \brief The fraction [0,1] of inlier points required for accepting a transformation */
	float inlier_fraction_;

	int inlier_number_;

	/** \brief Inlier points of final transformation as indices into source */
	std::vector<int> inliers_;
	std::vector<int> inliers_target_;

	std::vector<Eigen::Quaternionf> rots_;
	std::vector<Eigen::Vector3f> trans_;

	float angle_diff_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::setSourceFeatures (const FeatureCloudConstPtr &features)
{
	if (features == NULL || features->empty ())
	{
		PCL_ERROR ("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
		return;
	}
	input_features_ = features;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::setTargetFeatures (const FeatureCloudConstPtr &features)
{
	if (features == NULL || features->empty ())
	{
		PCL_ERROR ("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
		return;
	}
	target_features_ = features;
	feature_tree_->setInputCloud (target_features_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::selectSamples (
	const PointCloudSource &cloud, int nr_samples, std::vector<int> &sample_indices)
{
	if (nr_samples > static_cast<int> (cloud.points.size ()))
	{
		PCL_ERROR ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
		PCL_ERROR ("The number of samples (%d) must not be greater than the number of points (%lu)!\n",
			nr_samples, cloud.points.size ());
		return;
	}

	sample_indices.resize (nr_samples);
	int temp_sample;

	// Draw random samples until n samples is reached
	for (int i = 0; i < nr_samples; i++)
	{
		// Select a random number
		sample_indices[i] = getRandomIndex (static_cast<int> (cloud.points.size ()) - i);

		// Run trough list of numbers, starting at the lowest, to avoid duplicates
		for (int j = 0; j < i; j++)
		{
			// Move value up if it is higher than previous selections to ensure true randomness
			if (sample_indices[i] >= sample_indices[j])
			{
				sample_indices[i]++;
			}
			else
			{
				// The new number is lower, place it at the correct point and break for a sorted list
				temp_sample = sample_indices[i];
				for (int k = i; k > j; k--)
					sample_indices[k] = sample_indices[k - 1];

				sample_indices[j] = temp_sample;
				break;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::findSimilarFeatures (
	const std::vector<int> &sample_indices,
	std::vector<std::vector<int> >& similar_features,
	std::vector<int> &corresponding_indices)
{
	// Allocate results
	corresponding_indices.resize (sample_indices.size ());
	std::vector<float> nn_distances (k_correspondences_);

	// Loop over the sampled features
	for (size_t i = 0; i < sample_indices.size (); ++i)
	{
		// Current feature index
		const int idx = sample_indices[i];

		// Find the k nearest feature neighbors to the sampled input feature if they are not in the cache already
		if (similar_features[idx].empty ())
			feature_tree_->nearestKSearch (*input_features_, idx, k_correspondences_, similar_features[idx], nn_distances);

		// Select one at random and add it to corresponding_indices
		if (k_correspondences_ == 1)
			corresponding_indices[i] = similar_features[idx][0];
		else
			corresponding_indices[i] = similar_features[idx][getRandomIndex (k_correspondences_)];
	}
}

template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::getDenseCorrespondences (
	std::vector<int> &corres, std::vector<float> & dis2)
{
	// Allocate results
	corres.resize (input_features_->size ());
	dis2.resize( input_features_->size() );
	std::vector<float> nn_distances (1);
	std::vector<int> indices(1);

	// Loop over the sampled features
	for (size_t i = 0; i < input_features_->size (); ++i)
	{
		// Find the k nearest feature neighbors to the sampled input feature if they are not in the cache already
		feature_tree_->nearestKSearch (*input_features_, i, 1, indices, nn_distances);
		corres[ i ] = indices[ 0 ];
		dis2[ i ] = nn_distances[ 0 ];
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
	// Some sanity checks first
	if (!input_features_)
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("No source features were given! Call setSourceFeatures before aligning.\n");
		return;
	}
	if (!target_features_)
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("No target features were given! Call setTargetFeatures before aligning.\n");
		return;
	}

	if (input_->size () != input_features_->size ())
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
			input_->size (), input_features_->size ());
		return;
	}

	if (target_->size () != target_features_->size ())
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
			target_->size (), target_features_->size ());
		return;
	}

	if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f)
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("Illegal inlier fraction %f, must be in [0,1]!\n",
			inlier_fraction_);
		return;
	}

	const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold ();
	if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f)
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("Illegal prerejection similarity threshold %f, must be in [0,1[!\n",
			similarity_threshold);
		return;
	}

	if (k_correspondences_ <= 0)
	{
		PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
		PCL_ERROR ("Illegal correspondence randomness %d, must be > 0!\n",
			k_correspondences_);
		return;
	}

	// Initialize prerejector (similarity threshold already set to default value in constructor)
	correspondence_rejector_poly_->setInputSource (input_);
	correspondence_rejector_poly_->setInputTarget (target_);
	correspondence_rejector_poly_->setCardinality (nr_samples_);
	int num_rejections = 0; // For debugging
	int num_normal_rejections = 0;

	// Initialize results
	final_transformation_ = guess;
	inliers_.clear ();
	inliers_target_.clear();
	float lowest_error = std::numeric_limits<float>::max ();
	converged_ = false;

	// Temporaries
	std::vector<int> inliers;
	std::vector<int> inliers_target;
	float inlier_fraction;
	float error;

	// If guess is not the Identity matrix we check it
	if (!guess.isApprox (Eigen::Matrix4f::Identity (), 0.01f))
	{
		getFitness (inliers, inliers_target, error);
		inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
		error /= static_cast<float> (inliers.size ());

		if ( ( inlier_fraction >= inlier_fraction_ || inliers.size() > inlier_number_ ) && error < lowest_error)
		{
			inliers_ = inliers;
			inliers_target_ = inliers_target;
			lowest_error = error;
			converged_ = true;
		}
	}

	// Feature correspondence cache
	std::vector<std::vector<int> > similar_features (input_->size ());
			//std::vector< int > corres;
			//std::vector< float > dis2;
			//getDenseCorrespondences( corres, dis2 );

	// Start
	for (int i = 0; i < max_iterations_; ++i)
	{
		// Temporary containers
		std::vector<int> sample_indices;
		std::vector<int> corresponding_indices;

		// Draw nr_samples_ random samples
		selectSamples (*input_, nr_samples_, sample_indices);

		// Find corresponding features in the target cloud
		findSimilarFeatures (sample_indices, similar_features, corresponding_indices);

		// Apply prerejection
		if (!correspondence_rejector_poly_->thresholdPolygon (sample_indices, corresponding_indices))
		{
			++num_rejections;
			continue;
		}

		// Estimate the transform from the correspondences, write to transformation_
		if ( nr_samples_ > 2 ) {
			transformation_estimation_->estimateRigidTransformation (*input_, sample_indices, *target_, corresponding_indices, transformation_);
		} else {
			std::vector< int > idxs;
			idxs.push_back( 0 );
			idxs.push_back( 1 );
			idxs.push_back( 2 );
			idxs.push_back( 3 );

			PointCloudSource temp_input;
			PointCloudTarget temp_target;

			temp_input.push_back( ( *input_ )[ sample_indices[ 0 ] ] );
			temp_input.push_back( ( *input_ )[ sample_indices[ 1 ] ] );
			Eigen::Vector3d sn0( temp_input[ 0 ].normal_x, temp_input[ 0 ].normal_y, temp_input[ 0 ].normal_z );
			Eigen::Vector3d sn1( temp_input[ 1 ].normal_x, temp_input[ 1 ].normal_y, temp_input[ 1 ].normal_z );
			Eigen::Vector3d sn = sn0 + sn1;
			sn.normalize();
			Eigen::Vector3d sp0( temp_input[ 0 ].x, temp_input[ 0 ].y, temp_input[ 0 ].z );
			Eigen::Vector3d sp1( temp_input[ 1 ].x, temp_input[ 1 ].y, temp_input[ 1 ].z );
			Eigen::Vector3d sp = ( sp0 + sp1 ) * 0.5;
			Eigen::Vector3d sp2 = sp + ( sp1 - sp ).cross( sn );
			Eigen::Vector3d sp3 = sp - ( sp1 - sp ).cross( sn );
			
			PointSource tempp;
			tempp.normal_x = sn( 0 ); tempp.normal_y = sn( 1 ); tempp.normal_z = sn( 2 );
			tempp.x = sp2( 0 ); tempp.y = sp2( 1 ); tempp.z = sp2( 2 );
			temp_input.push_back( tempp );
			tempp.x = sp3( 0 ); tempp.y = sp3( 1 ); tempp.z = sp3( 2 );
			temp_input.push_back( tempp );

			temp_target.push_back( ( *target_ )[ corresponding_indices[ 0 ] ] );
			temp_target.push_back( ( *target_ )[ corresponding_indices[ 1 ] ] );
			Eigen::Vector3d tn0( temp_target[ 0 ].normal_x, temp_target[ 0 ].normal_y, temp_target[ 0 ].normal_z );
			Eigen::Vector3d tn1( temp_target[ 1 ].normal_x, temp_target[ 1 ].normal_y, temp_target[ 1 ].normal_z );
			Eigen::Vector3d tn = tn0 + tn1;
			tn.normalize();
			Eigen::Vector3d tp0( temp_target[ 0 ].x, temp_target[ 0 ].y, temp_target[ 0 ].z );
			Eigen::Vector3d tp1( temp_target[ 1 ].x, temp_target[ 1 ].y, temp_target[ 1 ].z );
			Eigen::Vector3d tp = ( tp0 + tp1 ) * 0.5;
			Eigen::Vector3d tp2 = tp + ( tp1 - tp ).cross( tn );
			Eigen::Vector3d tp3 = tp - ( tp1 - tp ).cross( tn );
			
			PointTarget tempt;
			tempt.normal_x = sn( 0 ); tempt.normal_y = sn( 1 ); tempt.normal_z = sn( 2 );
			tempt.x = sp2( 0 ); tempt.y = sp2( 1 ); tempt.z = sp2( 2 );
			temp_target.push_back( tempt );
			tempt.x = sp3( 0 ); tempt.y = sp3( 1 ); tempt.z = sp3( 2 );
			temp_target.push_back( tempt );

			transformation_estimation_->estimateRigidTransformation ( temp_input, idxs, temp_target, idxs, transformation_);

			if ( _isnanf( transformation_( 0, 0 ) ) ) {
				++num_normal_rejections;
				continue;
			}

			/*
			cout << idxs[ 0 ] << idxs[ 1 ] << idxs[ 2 ] << idxs[ 3 ] << endl << endl;
			cout << sn << endl << endl;
			cout << sp0 << endl;
			cout << sp1 << endl;
			cout << sp2 << endl;
			cout << sp3 << endl;
			cout << endl;

			cout << tn << endl << endl;
			cout << tp0 << endl;
			cout << tp1 << endl;
			cout << tp2 << endl;
			cout << tp3 << endl;
			cout << endl;

			cout << transformation_ << endl;
			boost::this_thread::sleep( boost::posix_time::milliseconds( 10000 ) );
			*/
		}

		if ( !thresholdNormal( sample_indices, corresponding_indices, transformation_ ) ) {
			++num_normal_rejections;
			continue;
		}

		// Take a backup of previous result
		const Eigen::Matrix4f final_transformation_prev = final_transformation_;

		// Set final result to current transformation
		final_transformation_ = transformation_;

		// Transform the input and compute the error (uses input_ and final_transformation_)
		getFitness (inliers, inliers_target, error);

		// Restore previous result
		final_transformation_ = final_transformation_prev;

		// If the new fit is better, update results
		inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());

		// Update result if pose hypothesis is better
		if ( ( inlier_fraction >= inlier_fraction_ || inliers.size() > inlier_number_ ) && error < lowest_error)
		{
			inliers_ = inliers;
			inliers_target_ = inliers_target;
			lowest_error = error;
			converged_ = true;
			final_transformation_ = transformation_;

			//printf( "(%d,%d), (%d,%d), (%d,%d)\n", corresponding_indices[ 0 ], sample_indices[ 0 ], corresponding_indices[ 1 ], sample_indices[ 1 ], corresponding_indices[ 2 ], sample_indices[ 2 ] );
			//printf( "%d - %.6f\n", corres[ sample_indices[ 0 ] ], dis2[ sample_indices[ 0 ] ] );
			//printf( "%d - %.6f\n", corres[ sample_indices[ 1 ] ], dis2[ sample_indices[ 1 ] ] );
			//printf( "%d - %.6f\n", corres[ sample_indices[ 2 ] ], dis2[ sample_indices[ 2 ] ] );
		}

		Eigen::Affine3f x( transformation_ );
		rots_.push_back( Eigen::Quaternionf( x.linear() ) );
		trans_.push_back( x.translation() );
	}

	// Apply the final transformation
	if (converged_)
		transformPointCloud (*input_, output, final_transformation_);

	// Debug output
	PCL_DEBUG("[pcl::%s::computeTransformation] Rejected (%i, %i) out of %i generated pose hypotheses (%i remains).\n",
		getClassName ().c_str (), num_rejections, num_normal_rejections, max_iterations_, max_iterations_ - num_rejections - num_normal_rejections);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::getFitness (std::vector<int>& inliers, std::vector<int> &inliers_target, float& fitness_score)
{
	// Initialize variables
	inliers.clear ();
	inliers.reserve (input_->size ());
	fitness_score = 0.0f;

	inliers_target.clear();
	inliers_target.reserve( target_->size() );

	// Use squared distance for comparison with NN search results
	const float max_range = corr_dist_threshold_ * corr_dist_threshold_;

	// Transform the input dataset using the final transformation
	PointCloudSource input_transformed;
	input_transformed.resize (input_->size ());
	transformPointCloud (*input_, input_transformed, final_transformation_);

	// For each point in the source dataset
	for (size_t i = 0; i < input_transformed.points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			// Update inliers
			inliers.push_back (static_cast<int> (i));
			inliers_target.push_back( nn_indices[ 0 ] );

			// Update fitness score
			fitness_score += nn_dists[0];
		}
	}

	// Calculate MSE
	if (inliers.size () > 0)
		fitness_score /= static_cast<float> (inliers.size ());
	else
		fitness_score = std::numeric_limits<float>::max ();
}

template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::getInformation()
{
	information_source_.setZero();
	information_target_.setZero();

	for ( int i = 0; i < ( int )inliers_.size(); i++ ) {
		const PointSource * source_it = &( input_->points[ inliers_[ i ] ] );
		const float & sx = source_it->x;
		const float & sy = source_it->y;
		const float & sz = source_it->z;
		Eigen::Matrix< double, 3, 6 > A;
		A << 1, 0, 0, 0, 2 * sz, - 2 * sy,
			0, 1, 0, - 2 * sz, 0, 2 * sx,
			0, 0, 1, 2 * sy, - 2 * sx, 0;
		information_source_ += A.transpose() * A;

		const PointTarget * target_it = &( target_->points[ inliers_target_[ i ] ] );
		const float & tx = target_it->x;
		const float & ty = target_it->y;
		const float & tz = target_it->z;
		Eigen::Matrix< double, 3, 6 > AA;
		AA << 1, 0, 0, 0, 2 * tz, - 2 * ty,
			0, 1, 0, - 2 * tz, 0, 2 * tx,
			0, 0, 1, 2 * ty, - 2 * tx, 0;
		information_target_ += AA.transpose() * AA;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
	RansacCurvature<PointSource, PointTarget, FeatureT>::writeAuxData( const std::string & filename )
{
	pcl::console::print_info( "%d trials are tested.\n", rots_.size() );
	FILE * f = fopen( filename.c_str(), "w" );

	for ( int i = 0; i < rots_.size(); i++ ) {
		fprintf( f, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", rots_[ i ].w(), rots_[ i ].x(), rots_[ i ].y(), rots_[ i ].z(), trans_[ i ]( 0 ), trans_[ i ]( 1 ), trans_[ i ]( 2 ) );
	}

	fclose( f );
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
	RansacCurvature<PointSource, PointTarget, Scalar>::align_redux( PointCloudSource &output, const Eigen::Matrix4f& guess )
{
	if (!initCompute ()) 
		return;

	// Resize the output dataset
	if (output.points.size () != indices_->size ())
		output.points.resize (indices_->size ());
	// Copy the header
	output.header   = input_->header;
	// Check if the output will be computed for all points or only a subset
	if (indices_->size () != input_->points.size ())
	{
		output.width    = static_cast<uint32_t> (indices_->size ());
		output.height   = 1;
	}
	else
	{
		output.width    = static_cast<uint32_t> (input_->width);
		output.height   = input_->height;
	}
	output.is_dense = input_->is_dense;

	// Copy the point data to output
	for (size_t i = 0; i < indices_->size (); ++i)
		output.points[i] = input_->points[(*indices_)[i]];

	// Perform the actual transformation computation
	converged_ = false;
	final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix4f::Identity ();

	// Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid 
	// transformation
	for (size_t i = 0; i < indices_->size (); ++i)
		output.points[i].data[3] = 1.0;

	// Initialize results
	final_transformation_ = guess;
	inliers_.clear ();
	inliers_target_.clear();
	float lowest_error = std::numeric_limits<float>::max ();
	converged_ = false;

	// Temporaries
	std::vector<int> inliers;
	std::vector<int> inliers_target;
	float inlier_fraction;
	float error;

	getFitness (inliers, inliers_target, error);
	inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
	error /= static_cast<float> (inliers.size ());

	if ( ( inlier_fraction >= inlier_fraction_ || inliers.size() > inlier_number_ ) && error < lowest_error)
	{
		inliers_ = inliers;
		inliers_target_ = inliers_target;
		lowest_error = error;
		converged_ = true;
	}

	// Apply the final transformation
	if (converged_)
		transformPointCloud (*input_, output, final_transformation_);

	deinitCompute ();
}
