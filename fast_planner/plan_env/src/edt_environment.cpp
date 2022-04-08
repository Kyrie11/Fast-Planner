/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include <plan_env/edt_environment.h>

namespace fast_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::init() {
}

void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

void EDTEnvironment::setObjPrediction(ObjPrediction prediction) {
  this->obj_prediction_ = prediction;
}

void EDTEnvironment::setObjScale(ObjScale scale) {
  this->obj_scale_ = scale;
}

double EDTEnvironment::distToBox(int idx, const Eigen::Vector3d& pos, const double& time) {
  // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
  Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

  Eigen::Vector3d box_max = pos_box + 0.5 * obj_scale_->at(idx);
  Eigen::Vector3d box_min = pos_box - 0.5 * obj_scale_->at(idx);

  Eigen::Vector3d dist;

  for (int i = 0; i < 3; i++) {
    dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)),
                                                                       fabs(pos(i) - box_max(i)));
  }

  return dist.norm();
}

double EDTEnvironment::minDistToAllBox(const Eigen::Vector3d& pos, const double& time) {
  double dist = 10000000.0;
  for (int i = 0; i < obj_prediction_->size(); i++) {
    double di = distToBox(i, pos, time);
    if (di < dist) dist = di;
  }

  return dist;
}

void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
      }
    }
  }
}

pair<double, Eigen::Vector3d> EDTEnvironment::interpolateTrilinear(double values[2][2][2],
                                                                   const Eigen::Vector3d& diff,
                                                                   double& value,
                                                                   Eigen::Vector3d& grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

pair<double, Eigen::Vector3d> EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                                                  double time, double& dist,
                                                                  Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinear(dists, diff, dist, grad);
}

double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d& pos, double time) {
  double d1 = sdf_map_->getDistance(pos);
  if (time < 0.0) {
    return d1;
  } else {
    double d2 = minDistToAllBox(pos, time);
    return min(d1, d2);
  }
}


double EDTEnvironment::lineCollision(Eigen::Vector3d& pos1, Eigen::Vector3d& pos2) {
  double resolution = sdf_map_->getResolution();
  double dx = pos2[0] - pos1[0];
  double dy = pos2[1] - pos1[1];
  double dz = pos2[2] - pos1[2];

  double ret = 100.0;
  if (abs(dx) >= abs(dy) && abs(dx) >= abs(dz)) {
    double yr = dy / dx * resolution;
    double zr = dz / dx * resolution;
    if (pos2[0] > pos1[0]) {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] + resolution;
      pos[1] = pos1[1] + yr;
      pos[2] = pos1[2] + zr;
      while (pos[0] < pos2[0]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] += resolution;
        pos[1] += yr;
        pos[2] += zr;
      }
    } else {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] - resolution;
      pos[1] = pos1[1] - yr;
      pos[2] = pos1[2] - zr;
      while (pos[0] > pos2[0]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] -= resolution;
        pos[1] -= yr;
        pos[2] -= zr;
      }
    }
  } else if (abs(dy) >= abs(dx) && abs(dy) >= abs(dz)) {
    double xr = dx / dy * resolution;
    double zr = dz / dy * resolution;
    if (pos2[1] > pos1[1]) {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] + xr;
      pos[1] = pos1[1] + resolution;
      pos[2] = pos1[2] + zr;
      while (pos[1] < pos2[1]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] += xr;
        pos[1] += resolution;
        pos[2] += zr;
      }
    } else {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] - xr;
      pos[1] = pos1[1] - resolution;
      pos[2] = pos1[2] - zr;
      while (pos[1] > pos2[1]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] -= xr;
        pos[1] -= resolution;
        pos[2] -= zr;
      }
    }
  } else {
    double xr = dx / dz * resolution;
    double yr = dy / dz * resolution;
    if (pos2[2] > pos1[2]) {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] + xr;
      pos[1] = pos1[1] + yr;
      pos[2] = pos1[2] + resolution;
      while (pos[2] < pos2[2]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] += xr;
        pos[1] += yr;
        pos[2] += resolution;
      }
    } else {
      Eigen::Vector3d pos;
      pos[0] = pos1[0] - xr;
      pos[1] = pos1[1] - yr;
      pos[2] = pos1[2] - resolution;
      while (pos[2] > pos2[2]) {
        ret = min(ret, sdf_map_->getDistance(pos));
        if (ret <= 0) {
          return -1.0;
        }
        pos[0] -= xr;
        pos[1] -= yr;
        pos[2] -= resolution;
      }
    }
  }

  return ret;
}
// EDTEnvironment::
}  // namespace fast_planner
