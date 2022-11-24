// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/pipelines/registration/Registration.h"

#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/registration/Feature.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"
#include "open3d/utility/Random.h"
#include "iostream"
#include <Eigen/Dense>
//#include <filesystem>
#include <fmt/format.h>
#include <memory>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/utility/Optional.h"

#include <phaser/common/point-types.h>


#include "open3d/Open3D.h"
#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"
#include <Eigen/Core>                   // for MatrixMap

#include <algorithm>                    // for copy_n, fill_n
#include <cstdint>                      // for uint8_t, uint32_t
#include <iostream>                      // for ostream, operator<<
#include <type_traits>                  // for enable_if_t    


namespace phaser_core {

DEFINE_string(
        target_cloud,
        "c:\\repo\\apricus\\phaser_test_data\\test_clouds\\os0\\target_4.ply",
        "Defines the path to the target cloud.");
DEFINE_string(
        source_cloud,
        "c:\\repo\\apricus\\phaser_test_data\\test_clouds\\os0\\source_4.ply",
        "Defines the path to the source cloud.");
DEFINE_string(reg_cloud,
              "c:\\repo\\apricus\\phaser_core\\reg_4.ply",
              "Defines the path to the registered cloud.");

// BAH, TBD:set these values to good defaults,
//          similarly(not identical) named inputs _spherical_bandwidth
//          in phaser core lib source, why?
DEFINE_int32(phaser_core_spherical_bandwidth,
             150,
             "spherical bandwidth");  // 150 original
DEFINE_int32(phaser_core_spherical_zero_padding, 10, "zero pad");
DEFINE_int32(phaser_core_spherical_low_pass_lower_bound,
             0,
             "low pass - lower band");
DEFINE_int32(phaser_core_spherical_low_pass_upper_bound,
             10000,
             "low pass - upper band");

DEFINE_int32(phaser_core_spatial_n_voxels, 201, "");
DEFINE_int32(phaser_core_spatial_discretize_lower, -50, "");
DEFINE_int32(phaser_core_spatial_discretize_upper, 50, "");
DEFINE_int32(phaser_core_spatial_zero_padding, 0, "");
DEFINE_int32(phaser_core_spatial_low_pass_lower_bound, 85, "");
DEFINE_int32(phaser_core_spatial_low_pass_upper_bound, 115, "");
// namespace phaser_core
}  // namespace phaser_core
namespace open3d {
namespace pipelines {
namespace registration {
// BAH, TODO, move next two helper functions to library. 
// copied from 👉🏽 aprikus project PointCloud.cpp
//      

geom::PointCloud &FixUpO3dColors(geom::PointCloud &pntCld) {
    double Scale = 1.0;
    for (auto &clr : pntCld.colors_) {
        double r = clr(0) * Scale;
        clr(1) = r;
        clr(2) = r;
        clr(0) = r;
    }
    return pntCld;
}

model::PointCloudPtr MakeModelCloud(geom::PointCloud gcld,
                                    double voxelSize = -1) {


    common::PointCloud_tPtr pntCldPntr(&FixUpO3dColors(gcld));

    // BAH, original test program did NOT have downsample.  So
    //  call MakeModelCloud withOUT voxelsize parameter,

    if (voxelSize > 0) pntCldPntr = pntCldPntr->VoxelDownSample(voxelSize);
    model::PointCloud *mCld = new model::PointCloud(pntCldPntr);

    model::PointCloudPtr mCldPtr(mCld);
    if (!mCldPtr->getRawCloud()->HasNormals()) {
        utility::ScopeTimer timer("Normal estimation with KNN10");
        for (int i = 0; i < 10; i++) {
            mCldPtr->getRawCloud()->EstimateNormals(
                    geom::KDTreeSearchParamKNN(10));
            mCldPtr->getRawCloud()->EstimateNormals(
                    geom::KDTreeSearchParamKNN(10));
        }
    }
   
    return mCldPtr;
}
static RegistrationResult GetRegistrationResultAndCorrespondences(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const geometry::KDTreeFlann &target_kdtree,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);
    if (max_correspondence_distance <= 0.0) {
        return result;
    }

    double error2 = 0.0;

#pragma omp parallel
    {
        double error2_private = 0.0;
        CorrespondenceSet correspondence_set_private;
#pragma omp for nowait
        for (int i = 0; i < (int)source.points_.size(); i++) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            const auto &point = source.points_[i];
            if (target_kdtree.SearchHybrid(point, max_correspondence_distance,
                                           1, indices, dists) > 0) {
                error2_private += dists[0];
                correspondence_set_private.push_back(
                        Eigen::Vector2i(i, indices[0]));
            }
        }
#pragma omp critical(GetRegistrationResultAndCorrespondences)
        {
            for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                result.correspondence_set_.push_back(
                        correspondence_set_private[i]);
            }
            error2 += error2_private;
        }
    }

    if (result.correspondence_set_.empty()) {
        result.fitness_ = 0.0;
        result.inlier_rmse_ = 0.0;
    } else {
        size_t corres_number = result.correspondence_set_.size();
        result.fitness_ = (double)corres_number / (double)source.points_.size();
        result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
    }
    return result;
}

static double EvaluateInlierCorrespondenceRatio(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);

    int inlier_corres = 0;
    double max_dis2 = max_correspondence_distance * max_correspondence_distance;
    for (const auto &c : corres) {
        double dis2 =
                (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
        if (dis2 < max_dis2) {
            inlier_corres++;
        }
    }

    return double(inlier_corres) / double(corres.size());
}

RegistrationResult EvaluateRegistration(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d
                &transformation /* = Eigen::Matrix4d::Identity()*/) {
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloud pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    return GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
}
//BAH, add Aprikus call here
// NOTE: to attach debugger
// go to Debug ➡️ 📎 attach  to process 
// 👉🏽 native code
// Also, start python o3d(python310) vm at 
// seperate command line. Run python test scipt 
// that calls  o3d.pipelines.registration.registration_icp(...)
// there should be TestAprikus.py (open3d\build)
RegistrationResult RegistrationICP(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/) {
    if (max_correspondence_distance <= 0.0) {
        utility::LogError("Invalid max_correspondence_distance.");
    }
    if ((estimation.GetTransformationEstimationType() ==
                 TransformationEstimationType::PointToPlane ||
         estimation.GetTransformationEstimationType() ==
                 TransformationEstimationType::ColoredICP) &&
        (!target.HasNormals())) {
        utility::LogError(
                "TransformationEstimationPointToPlane and "
                "TransformationEstimationColoredICP "
                "require pre-computed normal vectors for target PointCloud.");
    }
    if ((estimation.GetTransformationEstimationType() ==
         TransformationEstimationType::GeneralizedICP) &&
        (!target.HasCovariances() || !source.HasCovariances())) {
        utility::LogError(
                "TransformationEstimationForGeneralizedICP require "
                "pre-computed per point covariances matrices for source and "
                "target PointCloud.");
    }
    //BAH, new global registration, so tell user when selected ( can remove later)
    if ( (estimation.GetTransformationEstimationType() == TransformationEstimationType::Phaser ) ) {
        std::cout << "phaser global registration method" << std::endl;
    }
    Eigen::Matrix4d transformation = init;
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloud pcd = source;
    if (!init.isIdentity()) {
        pcd.Transform(init);
    }
    auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");

    //  
   // model::RegistrationResult res0 =
   //         ctrl->registerPointCloud(MakeModelCloud(target), MakeModelCloud(pcd));

    std::cout << "Registration: " << std::endl;
    RegistrationResult result;
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
    for (int i = 0; i < criteria.max_iteration_; i++) {
        utility::LogDebug("ICP Iteration #{:d}: Fitness {:.4f}, RMSE {:.4f}", i,
                          result.fitness_, result.inlier_rmse_);
        Eigen::Matrix4d update = estimation.ComputeTransformation(
                pcd, target, result.correspondence_set_);
        transformation = update * transformation;
        pcd.Transform(update);
        RegistrationResult backup = result;
        result = GetRegistrationResultAndCorrespondences(
                pcd, target, kdtree, max_correspondence_distance,
                transformation);
        if (std::abs(backup.fitness_ - result.fitness_) <
                    criteria.relative_fitness_ &&
            std::abs(backup.inlier_rmse_ - result.inlier_rmse_) <
                    criteria.relative_rmse_) {
            break;
        }
    }
    return result;
}

RegistrationResult RegistrationRANSACBasedOnCorrespondence(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres,
        double max_correspondence_distance,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers /* = {}*/,
        const RANSACConvergenceCriteria &criteria
        /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || (int)corres.size() < ransac_n ||
        max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    RegistrationResult best_result;
    geometry::KDTreeFlann kdtree(target);
    int est_k_global = criteria.max_iteration_;
    int total_validation = 0;

#pragma omp parallel
    {
        CorrespondenceSet ransac_corres(ransac_n);
        RegistrationResult best_result_local;
        int est_k_local = criteria.max_iteration_;
        utility::random::UniformIntGenerator<int> rand_gen(0,
                                                           corres.size() - 1);

#pragma omp for nowait
        for (int itr = 0; itr < criteria.max_iteration_; itr++) {
            if (itr < est_k_global) {
                for (int j = 0; j < ransac_n; j++) {
                    ransac_corres[j] = corres[rand_gen()];
                }

                Eigen::Matrix4d transformation =
                        estimation.ComputeTransformation(source, target,
                                                         ransac_corres);

                // Check transformation: inexpensive
                bool check = true;
                for (const auto &checker : checkers) {
                    if (!checker.get().Check(source, target, ransac_corres,
                                             transformation)) {
                        check = false;
                        break;
                    }
                }
                if (!check) continue;

                // Expensive validation
                geometry::PointCloud pcd = source;
                pcd.Transform(transformation);
                auto result = GetRegistrationResultAndCorrespondences(
                        pcd, target, kdtree, max_correspondence_distance,
                        transformation);

                if (result.IsBetterRANSACThan(best_result_local)) {
                    best_result_local = result;

                    double corres_inlier_ratio =
                            EvaluateInlierCorrespondenceRatio(
                                    pcd, target, corres,
                                    max_correspondence_distance,
                                    transformation);

                    // Update exit condition if necessary.
                    // If confidence is 1.0, then it is safely inf, we always
                    // consume all the iterations.
                    double est_k_local_d =
                            std::log(1.0 - criteria.confidence_) /
                            std::log(1.0 -
                                     std::pow(corres_inlier_ratio, ransac_n));
                    est_k_local =
                            est_k_local_d < est_k_global
                                    ? static_cast<int>(std::ceil(est_k_local_d))
                                    : est_k_local;
                    utility::LogDebug(
                            "Thread {:06d}: registration fitness={:.3f}, "
                            "corres inlier ratio={:.3f}, "
                            "Est. max k = {}",
                            itr, result.fitness_, corres_inlier_ratio,
                            est_k_local_d);
                }
#pragma omp critical
                {
                    total_validation += 1;
                    if (est_k_local < est_k_global) {
                        est_k_global = est_k_local;
                    }
                }
            }  // if
        }      // for loop

#pragma omp critical(RegistrationRANSACBasedOnCorrespondence)
        {
            if (best_result_local.IsBetterRANSACThan(best_result)) {
                best_result = best_result_local;
            }
        }
    }
    utility::LogDebug(
            "RANSAC exits after {:d} validations. Best inlier ratio {:e}, "
            "RMSE {:e}",
            total_validation, best_result.fitness_, best_result.inlier_rmse_);
    return best_result;
}

RegistrationResult RegistrationRANSACBasedOnFeatureMatching(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        bool mutual_filter,
        double max_correspondence_distance,
        const TransformationEstimation
                &estimation /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers /* = {}*/,
        const RANSACConvergenceCriteria
                &criteria /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    int num_src_pts = int(source.points_.size());
    int num_tgt_pts = int(target.points_.size());

    geometry::KDTreeFlann kdtree_target(target_feature);
    pipelines::registration::CorrespondenceSet corres_ij(num_src_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
    for (int i = 0; i < num_src_pts; i++) {
        std::vector<int> corres_tmp(1);
        std::vector<double> dist_tmp(1);

        kdtree_target.SearchKNN(Eigen::VectorXd(source_feature.data_.col(i)), 1,
                                corres_tmp, dist_tmp);
        int j = corres_tmp[0];
        corres_ij[i] = Eigen::Vector2i(i, j);
    }

    // Do reverse check if mutual_filter is enabled
    if (mutual_filter) {
        geometry::KDTreeFlann kdtree_source(source_feature);
        pipelines::registration::CorrespondenceSet corres_ji(num_tgt_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
        for (int j = 0; j < num_tgt_pts; ++j) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);
            kdtree_source.SearchKNN(
                    Eigen::VectorXd(target_feature.data_.col(j)), 1, corres_tmp,
                    dist_tmp);
            int i = corres_tmp[0];
            corres_ji[j] = Eigen::Vector2i(i, j);
        }

        pipelines::registration::CorrespondenceSet corres_mutual;
        for (int i = 0; i < num_src_pts; ++i) {
            int j = corres_ij[i](1);
            if (corres_ji[j](0) == i) {
                corres_mutual.emplace_back(i, j);
            }
        }

        // Empirically mutual correspondence set should not be too small
        if (int(corres_mutual.size()) >= ransac_n * 3) {
            utility::LogDebug("{:d} correspondences remain after mutual filter",
                              corres_mutual.size());
            return RegistrationRANSACBasedOnCorrespondence(
                    source, target, corres_mutual, max_correspondence_distance,
                    estimation, ransac_n, checkers, criteria);
        }
        utility::LogDebug(
                "Too few correspondences after mutual filter, fall back to "
                "original correspondences.");
    }

    return RegistrationRANSACBasedOnCorrespondence(
            source, target, corres_ij, max_correspondence_distance, estimation,
            ransac_n, checkers, criteria);
}

Eigen::Matrix6d GetInformationMatrixFromPointClouds(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    geometry::PointCloud pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    RegistrationResult result;
    geometry::KDTreeFlann target_kdtree(target);
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, target_kdtree, max_correspondence_distance,
            transformation);

    // write q^*
    // see http://redwood-data.org/indoor/registration.html
    // note: I comes first in this implementation
    Eigen::Matrix6d GTG = Eigen::Matrix6d::Zero();
#pragma omp parallel
    {
        Eigen::Matrix6d GTG_private = Eigen::Matrix6d::Zero();
        Eigen::Vector6d G_r_private = Eigen::Vector6d::Zero();
#pragma omp for nowait
        for (int c = 0; c < int(result.correspondence_set_.size()); c++) {
            int t = result.correspondence_set_[c](1);
            double x = target.points_[t](0);
            double y = target.points_[t](1);
            double z = target.points_[t](2);
            G_r_private.setZero();
            G_r_private(1) = z;
            G_r_private(2) = -y;
            G_r_private(3) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = -z;
            G_r_private(2) = x;
            G_r_private(4) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = y;
            G_r_private(1) = -x;
            G_r_private(5) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
        }
#pragma omp critical(GetInformationMatrixFromPointClouds)
        { GTG += GTG_private; }
    }
    return GTG;
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
