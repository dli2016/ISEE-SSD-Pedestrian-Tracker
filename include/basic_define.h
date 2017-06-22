/**@file    basic_define.h
 * @brief   Basic definitions for object tracking.
 * @version 0.3.1   2016/09/20,   0.3.2   2016/09/21
 *          Mainly modify vector to array; and delete member of 'video_url'. 
 * @email   da.li@cripac.ia.ac.cn
 *
 * Licensed under The MIT License [see LICENSE for details]
 */

#ifndef _BASIC_DEFINE_H_
#define _BASIC_DEFINE_H_

// #include <vector>
// #include <string>
// using namespace std;

#define VIDEO_URL_LEN 2048

/**
 * @struct BoundingBox.
 * @brief Bounding box of a pedestrian.
 * Each bounding box represents the location as well as the size
 * of a pedestrian in a frame.
 */
typedef struct _bb_t {
  int x;   // Left
  int y;   // Top
  int width;
  int height;
  unsigned char *patch_data;
} BoundingBox;

/**
 * @struct Trajectory
 * @brief Trajectory of a target pedestrian.
 * A tracklet represents a series of appears of a target pedestrian
 * in a video.
 */
typedef struct _trajectory_t {
  int id;
  int traj_size;
  char video_url[VIDEO_URL_LEN];
  int start_frame_idx;
  // BBList location_sequence;
  BoundingBox *location_sequence;
} Trajectory;

/**
 * @enum DetectionMethods
 */
enum DetectionMethods {
    MIN_METHOD = -1,
    MOTION_DETECTION = 0,
    SSD = 1,
    MAX_METHOD = 2,
};

#endif // _BASIC_DEFINE_H_
