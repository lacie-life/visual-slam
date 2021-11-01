/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAP_H_
#define MAP_H_

#include <mutex>
#include <set>

#include "KeyFrame.h"
#include "MapPoint.h"
namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;

class Map {
 public:
  Map();

  void AddKeyFrame(KeyFrame* keyframe);
  void AddMapPoint(MapPoint* map_point);
  void EraseMapPoint(MapPoint* map_point);
  void EraseKeyFrame(KeyFrame* KeyFrame);
  void SetReferenceMapPoints(const std::vector<MapPoint*>& map_points);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  std::vector<KeyFrame*> GetAllKeyFrames();
  std::vector<MapPoint*> GetAllMapPoints();
  std::vector<MapPoint*> GetReferenceMapPoints();

  long unsigned int MapPointsInMap();
  long unsigned KeyFramesInMap();

  long unsigned int GetMaxKFid();

  void clear();

  std::vector<KeyFrame*> keyframe_origins_;

  std::mutex mutex_map_update_;

  // This avoid that two points are created simultaneously in separate threads
  // (id conflict)
  std::mutex mutex_point_creation_;

 protected:
  std::set<MapPoint*> map_points_;
  std::set<KeyFrame*> keyframes_;

  std::vector<MapPoint*> reference_map_points_;

  long unsigned int max_keyframe_id_;

  // Index related to a big change in the map (loop closure, global BA)
  int big_change_index_;

  std::mutex mutex_map_;
};
}  // namespace ORB_SLAM2

#endif
