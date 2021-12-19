/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <bitset>
#include <set>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "camera_models.h"
#include "common_types.h"

void computeEssential(const Sophus::SE3d& T_0_1, Eigen::Matrix3d& E) {
  const Eigen::Vector3d t_0_1 = T_0_1.translation();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  // TODO: compute essential matrix

}

void findInliersEssential(const KeypointsData& kd1, const KeypointsData& kd2,
                          const std::shared_ptr<AbstractCamera<double>>& cam1,
                          const std::shared_ptr<AbstractCamera<double>>& cam2,
                          const Eigen::Matrix3d& E,
                          double epipolar_error_threshold, MatchData& md) {
  md.inliers.clear();

  for (size_t j = 0; j < md.matches.size(); j++) {
    const Eigen::Vector2d p0_2d = kd1.corners[md.matches[j].first];
    const Eigen::Vector2d p1_2d = kd2.corners[md.matches[j].second];

    // TODO: determine inliers and store in md.inliers
    

  }
}

void findInliersRansac(const KeypointsData& kd1, const KeypointsData& kd2,
                       const std::shared_ptr<AbstractCamera<double>>& cam1,
                       const std::shared_ptr<AbstractCamera<double>>& cam2,
                       double ransac_thresh, int ransac_min_inliers,
                       MatchData& md) {
  md.inliers.clear();

  // TODO: run RANSAC with using opengv's CentralRelativePose and
  // store in md.inliers. If the number if inliers is smaller than
  // ransac_min_inliers, leave md.inliers empty.

  opengv::bearingVectors_t bearingVectors1;
  opengv::bearingVectors_t bearingVectors2;

  for (size_t j = 0; j < md.matches.size(); j++) {
    const Eigen::Vector2d p0_2d = kd1.corners[md.matches[j].first];
    const Eigen::Vector2d p1_2d = kd2.corners[md.matches[j].second];

    Eigen::Vector3d p0_3d = cam1->unproject(p0_2d);
    Eigen::Vector3d p1_3d = cam2->unproject(p1_2d);

    bearingVectors1.push_back(p0_3d.normalized());
    bearingVectors2.push_back(p1_3d.normalized());

  }

      // create the central relative adapter
      opengv::relative_pose::CentralRelativeAdapter adapter(bearingVectors1, bearingVectors2);
      // create a RANSAC object
      opengv::sac::Ransac<opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
      // create a CentralRelativePoseSacProblem
      // (set algorithm to STEWENIUS, NISTER, SEVENPT, or EIGHTPT)
      std::shared_ptr<opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
          relposeproblem_ptr(
          new opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem(
          adapter,
          opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER ) );
      // run ransac
      ransac.sac_model_ = relposeproblem_ptr;
      ransac.threshold_ = ransac_thresh;
      ransac.max_iterations_ = 100;
      ransac.computeModel();
      // get the result
      opengv::transformation_t best_transformation =
          ransac.model_coefficients_;

      // non-linear optimization (using all available correspondences)

      Eigen::Vector3d t_0_1 = best_transformation.topRightCorner(3, 1);
      Eigen::Matrix3d R_0_1 = best_transformation.topLeftCorner(3, 3);

      adapter.sett12(t_0_1);
      adapter.setR12(R_0_1);

      opengv::transformation_t nonlinear_transformation =
          opengv::relative_pose::optimize_nonlinear(adapter, ransac.inliers_);

      ransac.sac_model_->selectWithinDistance(ransac.model_coefficients_, ransac.threshold_, ransac.inliers_);

      Sophus::SE3d test(nonlinear_transformation.topLeftCorner(3, 3), nonlinear_transformation.topRightCorner(3, 1));
      md.T_i_j = test;

      for (size_t j = 0; j < ransac.inliers_.size(); j++) {
          md.inliers.push_back(std::make_pair(md.matches[ransac.inliers_[j]].first, md.matches[ransac.inliers_[j]].second));
      }

        if (md.matches.size() < ransac_min_inliers) {
            md.matches.clear();
        }
}
