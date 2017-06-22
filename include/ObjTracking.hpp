
/** \file    tracking_inf.h
 *  \brief   The interface for object tracking.
 *  \version 0.1         0.2 (SSD for object detection)
 *  \date    2016-07-26  2017-05-24
 *  \email   da.li@cripac.ia.ac.cn
 *
 * Licensed under The MIT License [see LICENSE for details]
 */

#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <string>

#include "basic_define.h"

/**
 * @struct ObjTrackingParams
 */
typedef struct _obj_tracking_params_t {
    int w;                      // width of a video frame.
    int h;                      // height of a video frame.
    int chns_num;               // number of channels of a video frame.
    int detection_method;       // method to detect object.
    int tracking_method;        // method to accociate objects.
    int gpu_index;              // specify the gpu id (-1 for cpu only).
    int buffer_len;             // length of the configure buffer.
    float confidence_threshold; // threshold to make a sample positive.
    const char* buffer;         // the data of configure file
    const char* proto_path;     // the path of the file stores the Net architecture.
    const char* weights_path;   // the path of the trained model (weights).
} ObjTrackingParams;

/** \class ObjTacking
 *  \brief The implementation of object tracking:
 *         init(), doTrack(), destroy().
 */
class ObjTracking {
 public:
  ObjTracking() {
    handle = 0;
  }
  ~ObjTracking() {
    destroy();
  }

  /** \fn        init
   *  \brief     initialize the basic information of the input video and
   *             necessary parameters.
   *  \param[IN] params - parameters used in object tracking.
   *  \return    Whether the function run successfully or not.
   */
  bool init(const ObjTrackingParams& params);

  /** \fn         doTrack
   *  \brief      Start to track.
   *  \param[IN]  frame_data - rgb data of current video frame.
   *  \return     Whether the function run successfully or not.
   */
  bool doTrack(const unsigned char *frame_data);

  /**
   * \fn          getTrajs
   * \brief       To return the tracking results include the
   *              bounding boxes and rgb data.
   * \param[OUT]  trajs_num - the number of trajectories.
   * \return      A pointer points to trajectory data (see basic_define.h).
   *              It's not necessary to free the pointer, but you should
   *              assign NULL to it finally (see unittest/main.cpp).
   */
  Trajectory *getTrajs(int &trajs_num);

  /** \fn    destroy
   *  \brief Release recourses.
   */
  void destroy(void);

// Modified by da.li on 2016/11/9
 private:
  long handle;
// End

};  // ObjTracking

#endif  // _TRACKING_H_
