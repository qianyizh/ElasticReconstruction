#include "StdAfx.h"
#include "OptApp.h"
#include <boost/filesystem.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "vertigo/vertex_switchLinear.h"
#include "vertigo/edge_switchPrior.h"
#include "vertigo/edge_se3Switchable.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

COptApp::COptApp(void)
{
}

COptApp::~COptApp(void)
{
}

bool COptApp::Init()
{
	namespace fs = boost::filesystem;
	if ( fs::exists( fs::path( odometry_log_file_ ) ) ) {
		odometry_traj_.LoadFromFile( odometry_log_file_ );
		if ( fs::exists( fs::path( odometry_info_file_ ) ) ) {
			odometry_info_.LoadFromFile( odometry_info_file_ );
		}
	}
	if ( fs::exists( fs::path( loop_log_file_ ) ) ) {
		loop_traj_.LoadFromFile( loop_log_file_ );
		if ( fs::exists( fs::path( loop_info_file_ ) ) ) {
			loop_info_.LoadFromFile( loop_info_file_ );
		}
	}
	pose_traj_.data_.clear();
	pose_traj_.data_.push_back( FramedTransformation( 0, 0, 1, Eigen::Matrix4d::Identity() ) );
	for ( int i = 0; i < ( int )odometry_traj_.data_.size(); i++ ) {
		pose_traj_.data_.push_back( FramedTransformation( i + 1, i + 1, i + 2, pose_traj_.data_[ i ].transformation_ * odometry_traj_.data_[ i ].transformation_ ) );
	}
	return ( odometry_traj_.data_.size() > 0 );
}

void COptApp::OptimizeSwitchable()
{
	struct SwitchableEdge {
	public:
		VertexSwitchLinear * v_;
		EdgeSwitchPrior * ep_;
		EdgeSE3Switchable * e_;
		FramedTransformation * t_;
	};

    g2o::SparseOptimizer* optimizer;
	optimizer = new g2o::SparseOptimizer();
	optimizer->setVerbose(true);
	SlamBlockSolver * solver = NULL;
	SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
	linearSolver->setBlockOrdering(false);
	solver = new SlamBlockSolver(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(solver);
	optimizer->setAlgorithm(algo);

	std::vector< SwitchableEdge > switch_edge;

	Eigen::Matrix< double, 6, 6 > default_information;
	default_information = Eigen::Matrix< double, 6, 6 >::Identity();

	for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
		g2o::VertexSE3 * v = new g2o::VertexSE3();
		v->setId( i );
		v->setEstimate( Eigen2G2O( pose_traj_.data_[ i ].transformation_ ) );
		if ( i == 0 ) {
			v->setFixed( true );
		}
		optimizer->addVertex( v );

		if ( i > 0 ) {
			g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3();
			g2o_edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( i - 1 ));
			g2o_edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( i ));
			g2o_edge->setMeasurement( g2o::internal::fromSE3Quat( Eigen2G2O( odometry_traj_.data_[ i - 1 ].transformation_ ) ) );
			if ( odometry_info_.data_.size() > 0 ) {
				g2o_edge->setInformation( odometry_info_.data_[ i - 1 ].information_ );
			} else {
				g2o_edge->setInformation( default_information );
			}
			optimizer->addEdge( g2o_edge );
		}
	}

	for ( int i = 0; i < ( int )loop_traj_.data_.size(); i++ ) {
		FramedTransformation & t = loop_traj_.data_[ i ];

		SwitchableEdge edge;
		edge.t_ = &t;

		edge.v_ = new VertexSwitchLinear();
		edge.v_->setId( optimizer->vertices().size() );
		edge.v_->setEstimate( 1.0 );
		optimizer->addVertex( edge.v_ );

		edge.ep_ = new EdgeSwitchPrior();
		edge.ep_->vertices()[0] = edge.v_;
		edge.ep_->setMeasurement( 1.0 );
		edge.ep_->setInformation( Eigen::Matrix<double,1,1>::Identity() * weight_ );
		optimizer->addEdge( edge.ep_ );

		edge.e_ = new EdgeSE3Switchable();
		edge.e_->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( t.id1_ ));
		edge.e_->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( t.id2_ ));
		edge.e_->vertices()[2] = edge.v_;
		edge.e_->setMeasurement( g2o::internal::fromSE3Quat( Eigen2G2O( t.transformation_ ) ) );
		if ( loop_info_.data_.size() > 0 ) {
			edge.e_->setInformation( loop_info_.data_[ i ].information_ );
		} else {
			edge.e_->setInformation( default_information );
		}
		optimizer->addEdge( edge.e_ );
		switch_edge.push_back( edge );
	}

	optimizer->initializeOptimization();
	optimizer->optimize( max_iteration_ );

	for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
		g2o::VertexSE3 * v = dynamic_cast< g2o::VertexSE3 * >( optimizer->vertex( i ) );
		pose_traj_.data_[ i ].transformation_ = G2O2Matrix4d( v->estimateAsSE3Quat() );
	}
	pose_traj_.SaveToFile( pose_log_file_ );

	loop_remain_traj_.data_.clear();
	for ( int i = 0; i < ( int )switch_edge.size(); i++ ) {
		SwitchableEdge & edge = switch_edge[ i ];
		if ( edge.v_->estimate() > 0.5 ) {
			loop_remain_traj_.data_.push_back( loop_traj_.data_[ i ] );
		}
	}
	loop_remain_traj_.SaveToFile( loop_remain_log_file_ );
}

void COptApp::OptimizeEM()
{
	struct SwitchableEdge {
	public:
		double weight_;
		g2o::EdgeSE3 * e_;
		FramedTransformation * t_;
	};

	g2o::SparseOptimizer* optimizer;
	optimizer = new g2o::SparseOptimizer();
	optimizer->setVerbose(true);
	SlamBlockSolver * solver = NULL;
	SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
	linearSolver->setBlockOrdering(false);
	solver = new SlamBlockSolver(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(solver);
	optimizer->setAlgorithm(algo);

	std::vector< SwitchableEdge > switch_edge;

	Eigen::Matrix< double, 6, 6 > default_information;
	default_information = Eigen::Matrix< double, 6, 6 >::Identity();

	for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
		g2o::VertexSE3 * v = new g2o::VertexSE3();
		v->setId( i );
		v->setEstimate( Eigen2G2O( pose_traj_.data_[ i ].transformation_ ) );
		if ( i == 0 ) {
			v->setFixed( true );
		}
		optimizer->addVertex( v );

		if ( i > 0 ) {
			g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3();
			g2o_edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( i - 1 ));
			g2o_edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( i ));
			g2o_edge->setMeasurement( g2o::internal::fromSE3Quat( Eigen2G2O( odometry_traj_.data_[ i - 1 ].transformation_ ) ) );
			if ( odometry_info_.data_.size() > 0 ) {
				g2o_edge->setInformation( odometry_info_.data_[ i - 1 ].information_ );
			} else {
				g2o_edge->setInformation( default_information );
			}
			optimizer->addEdge( g2o_edge );
		}
	}

	for ( int i = 0; i < ( int )loop_traj_.data_.size(); i++ ) {
		FramedTransformation & t = loop_traj_.data_[ i ];
		SwitchableEdge edge;
		edge.t_ = &t;
		edge.weight_ = 0.0;

		edge.e_ = new g2o::EdgeSE3();
		edge.e_->vertices()[0] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( t.id1_ ));
		edge.e_->vertices()[1] = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex( t.id2_ ));
		edge.e_->setMeasurement( g2o::internal::fromSE3Quat( Eigen2G2O( t.transformation_ ) ) );
		if ( loop_info_.data_.size() > 0 ) {
			edge.e_->setInformation( loop_info_.data_[ i ].information_ );
		} else {
			edge.e_->setInformation( default_information );
		}
		optimizer->addEdge( edge.e_ );
		switch_edge.push_back( edge );
	}

	for ( int itr = 0; itr < max_iteration_; itr++ ) {
		// E step
		for ( int i = 0; i < ( int )switch_edge.size(); i++ ) {
			SwitchableEdge & edge = switch_edge[ i ];
			if ( loop_info_.data_.size() > 0 ) {
				edge.e_->setInformation( loop_info_.data_[ i ].information_ );
			} else {
				edge.e_->setInformation( default_information );
			}
			edge.e_->computeError();
			edge.weight_ = ( weight_ * weight_ ) / ( weight_ * weight_ + switch_edge[ i ].e_->chi2() );

			if ( loop_info_.data_.size() > 0 ) {
				edge.e_->setInformation( loop_info_.data_[ i ].information_ * sqrt( edge.weight_ ) );
			} else {
				edge.e_->setInformation( default_information * sqrt( edge.weight_ ) );
			}
		}

		// M step
		optimizer->initializeOptimization();
		optimizer->optimize( 1 );
	}

	for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
		g2o::VertexSE3 * v = dynamic_cast< g2o::VertexSE3 * >( optimizer->vertex( i ) );
		pose_traj_.data_[ i ].transformation_ = G2O2Matrix4d( v->estimateAsSE3Quat() );
	}
	pose_traj_.SaveToFile( pose_log_file_ );

	loop_remain_traj_.data_.clear();
	for ( int i = 0; i < ( int )switch_edge.size(); i++ ) {
		SwitchableEdge & edge = switch_edge[ i ];
		if ( edge.weight_ > 0.25 ) {
			loop_remain_traj_.data_.push_back( loop_traj_.data_[ i ] );
		}
	}
	loop_remain_traj_.SaveToFile( loop_remain_log_file_ );
}