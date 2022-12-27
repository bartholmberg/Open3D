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

#include "open3d/pipelines/registration/TransformationEstimation.h"

#include <Eigen/Geometry>

#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Eigen.h"
#include "iostream"

#include <Eigen/Dense>
// #include <filesystem>
#include <fmt/format.h>
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <phaser/common/point-types.h>

#include <Eigen/Core>  // for MatrixMap
#include <algorithm>   // for copy_n, fill_n
#include <cstdint>     // for uint8_t, uint32_t
#include <iostream>    // for ostream, operator<<
#include <memory>
#include <type_traits>  // for enable_if_t

//#include "open3d/Open3D.h"
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/utility/Optional.h"
//#include "phaser/backend/registration/sph-opt-registration.h"
#include "phaser/controller/cloud-controller.h"
#include "phaser/controller/TapPoint.h"
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

model::PointCloudPtr MakeModelCloud(geom::PointCloud agcld,
                                    double voxelSize = -1) {
    geom::PointCloud *gcld = new geom::PointCloud(agcld);
    common::PointCloud_tPtr pntCldPntr(&FixUpO3dColors(*gcld));

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
double TransformationEstimationPointToPoint::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return 0.0;
    double err = 0.0;
    for (const auto &c : corres) {
        err += (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
    }
    return std::sqrt(err / (double)corres.size());
}

Eigen::Matrix4d TransformationEstimationPointToPoint::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return Eigen::Matrix4d::Identity();
    Eigen::MatrixXd source_mat(3, corres.size());
    Eigen::MatrixXd target_mat(3, corres.size());
    for (size_t i = 0; i < corres.size(); i++) {
        source_mat.block<3, 1>(0, i) = source.points_[corres[i][0]];
        target_mat.block<3, 1>(0, i) = target.points_[corres[i][1]];
    }
    return Eigen::umeyama(source_mat, target_mat, with_scaling_);
}
// BAH, OLD NOTES move comments from Registration.cpp to here with the code.
//      keep for update plan.

// BAH, add Aprikus call here
//  NOTE: to attach debugger
//  go to Debug ➡️ 📎 attach  to process
//  👉🏽 native code
//  Also, start python o3d(python310) vm at
//  seperate command line. Run python test scipt
//  that calls  o3d.pipelines.registration.registration_icp(...)
//  there should be TestAprikus.py (open3d\build)
//  NOTE:  w/Aprikus linked in- Make Sure you have all the dll
//         either in same dir (e.g.- vm site-packages/open3d)
//         or added to the system path
//         ( fftw3.dll,
//         libfftw3-3.dll,libcrypto-3-x64.dll,\zlib1.dll,libssl-3-x64.dll,nlopt.dll,opencv_core3.dll)
//  NOTE: UPDATE, phaser moved to standard compute function in
//  TransformationEstimation.cpp// BAH, stub out for out for now. Calling phaser registration directly.
// 
//      BAH, NEW NOTES add Aprikus call here
Eigen::Matrix4d TransformationEstimationPhaser::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {


    // BAH, 12/4 got a crash here instantiating the CloudController
    //      ??? .  Pick it up here after making chili
    auto ctrl = std::make_unique<phaser_core::CloudController>("sph-opt");
    model::PointCloudPtr s0 = MakeModelCloud(source);
    model::PointCloudPtr t0 = MakeModelCloud(target);
   // model::RegistrationResult res0 = ctrl->registerPointCloud(t0, s0);

    // BAH, 🎚️ 🛸 🍯 add support for tap-off points to phaser.  Use variant return
    // type,  and tap point selector 🎚️ input 
    phaser_core::RegistrationResult vresult = ctrl->registerPointCloud(
            t0, s0, phaser_core::TapPoint::fullRegistration);
    auto res0 = std::get<model::RegistrationResult>(vresult);
    std::cout << "Registration: " << std::endl;
    common::PointCloud_tPtr b = res0.getRegisteredCloud()->getRawCloud();
    int N=b->points_.size();
    Eigen::MatrixXd resmat(3, N);
    for (size_t i = 0; i < N; i++) {
        resmat.block<3, 1>(0, i) = b->points_[i];
    } 

    Eigen::Vector3d rota = res0.getRotation();
    // BAH, ⛏️ transformation (rot) matrix between Phaser and o3d(below) 
    //          is 📎 different.  Sign difference in Tz
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = geometry::Geometry3D::GetRotationMatrixFromXYZ( {rota[0], rota[1], -rota[2]});
    std::cout << "\n 🐢 o3d Bingham rotation: " << rota.transpose() * 180.0 / M_PI << std::endl;
    T.block<3, 1>(0, 3) = res0.getTranslation();
    return T;
}
//  BAH, copy ICP version, should work fine for PHASER
//       TODO: verify
double TransformationEstimationPhaser::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals()) return 0.0;
    double err = 0.0, r;
    for (const auto &c : corres) {
        r = (source.points_[c[0]] - target.points_[c[1]])
                    .dot(target.normals_[c[1]]);
        err += r * r;
    }
    return std::sqrt(err / (double)corres.size());
}


double TransformationEstimationPointToPlane::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals()) return 0.0;
    double err = 0.0, r;
    for (const auto &c : corres) {
        r = (source.points_[c[0]] - target.points_[c[1]])
                    .dot(target.normals_[c[1]]);
        err += r * r;
    }
    return std::sqrt(err / (double)corres.size());
}

Eigen::Matrix4d TransformationEstimationPointToPlane::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || !target.HasNormals())
        return Eigen::Matrix4d::Identity();

    auto compute_jacobian_and_residual = [&](int i, Eigen::Vector6d &J_r,
                                             double &r, double &w) {
        const Eigen::Vector3d &vs = source.points_[corres[i][0]];
        const Eigen::Vector3d &vt = target.points_[corres[i][1]];
        const Eigen::Vector3d &nt = target.normals_[corres[i][1]];
        r = (vs - vt).dot(nt);
        w = kernel_->Weight(r);
        J_r.block<3, 1>(0, 0) = vs.cross(nt);
        J_r.block<3, 1>(3, 0) = nt;
    };

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    double r2;
    std::tie(JTJ, JTr, r2) =
            utility::ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                    compute_jacobian_and_residual, (int)corres.size());

    bool is_success;
    Eigen::Matrix4d extrinsic;
    std::tie(is_success, extrinsic) =
            utility::SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);

    return is_success ? extrinsic : Eigen::Matrix4d::Identity();
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
