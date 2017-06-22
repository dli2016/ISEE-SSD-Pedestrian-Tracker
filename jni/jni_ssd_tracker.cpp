/** \file    jni_ssd_tracker.cpp
 *  \brief   Implementation of JNI wrapper for object tracking.
 *  \version 0.1  modified by da.li on 2017/05/27 for ssd.
 *  \date    2016-07-26
 *  \email   da.li@cripac.ia.ac.cn kyu_115s@hotmail.com
 *
 * Licensed under The MIT License [see LICENSE for details]
 */

#include <cstdlib>
#include <cstdio>
#include <string>
#include <jni.h>

#include "jni_ssd_tracker.h"
#include "ObjTracking.hpp"

using namespace std;

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    initialize
 * Signature: (Lorg/cripac/isee/alg/pedestrian/tracking/SSDTracker/SSDTrackerParams;)J
 */
JNIEXPORT jlong JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_initialize
  (JNIEnv *env, jobject obj, jobject j_params) {

  jclass params_class = env->FindClass("org/cripac/isee/alg/pedestrian/tracking/SSDTracker$SSDTrackerParams");
  if (params_class == NULL) {
    fprintf(stderr, 
      "Error: Cannot find Java class: org/cripac/isee/alg/pedestrian/tracking/SSDTracker$SSDTrackerParams");
    fflush(stdout), fflush(stderr);
    return (jlong)NULL;
  }
  // Get Field.
  // Width.
  jfieldID width_field  = env->GetFieldID(params_class, "width", "I");
  // Height.
  jfieldID height_field = env->GetFieldID(params_class, "height", "I");
  // Number of Channels
  jfieldID channel_field= env->GetFieldID(params_class, "numChannels", "I");
  // GPU index.
  jfieldID gpu_index_field = env->GetFieldID(params_class, "gpuIndex", "I");
  // Buffer of Configure File.
  jfieldID conf_field = env->GetFieldID(params_class, "conf", "[B");
  // Confidence Threshold.
  jfieldID confidence_threshold_field = 
    env->GetFieldID(params_class, "confidenceThreshold", "F");
  // Path of protobuf file.
  jfieldID pb_path_field = 
    env->GetFieldID(params_class, "pbPath", "Ljava/lang/String;");
  // Path of caffe model.
  jfieldID model_path_field = 
    env->GetFieldID(params_class, "modelPath", "Ljava/lang/String;");

  ObjTrackingParams params;
  // Get Values.
  params.w = env->GetIntField(j_params, width_field);
  params.h = env->GetIntField(j_params, height_field);
  params.chns_num = env->GetIntField(j_params, channel_field);
  params.gpu_index= env->GetIntField(j_params, gpu_index_field);
  params.confidence_threshold = env->GetFloatField(j_params, confidence_threshold_field);
  params.detection_method = SSD;
  // Configure.
  jbyteArray jbytes_conf = (jbyteArray)(env->GetObjectField(j_params, conf_field));
  // Get number of bytes of the configuration file.
  jsize buffer_len = (env)->GetArrayLength(jbytes_conf);
  // Create a native buffer for the configuration file.
  // Retrieve bytes from Java to local.
  jbyte *conf_buffer = env->GetByteArrayElements(jbytes_conf, NULL);
  params.buffer = (const char*)conf_buffer;
  params.buffer_len = buffer_len;
  
  // Path.
  jstring jstring_pb_path = (jstring)(env->GetObjectField(j_params, pb_path_field));
  const int pb_len = env->GetStringUTFLength(jstring_pb_path);
  char* c_pb_path = new char[pb_len + 1];
  env->GetStringUTFRegion(jstring_pb_path, 0, pb_len, c_pb_path);
  c_pb_path[pb_len] = '\0';
  params.proto_path = c_pb_path;
  jstring jstring_model_path = (jstring)(env->GetObjectField(j_params, model_path_field));
  const int model_len = env->GetStringUTFLength(jstring_model_path);
  char* c_model_path = new char[model_len + 1];
  env->GetStringUTFRegion(jstring_model_path, 0, model_len, c_model_path);
  c_model_path[model_len] = '\0';
  params.weights_path = c_model_path;

  // Initialize a tracker with bytes of the given configuration file.
  ObjTracking *tracker = new ObjTracking;
  int res = tracker->init(params);
  env->ReleaseByteArrayElements(jbytes_conf, conf_buffer, 0);

  delete[] c_pb_path;
  c_pb_path = NULL;
  delete[] c_model_path;
  c_model_path = NULL;

  if (!res) {
    fprintf(stderr, "Error: The tracker initialization FAILED!\n");
    fflush(stdout), fflush(stderr);
    return (jlong)NULL;
  } else {
    fflush(stdout), fflush(stderr);
    return (jlong)tracker;
  }
}

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    feedFrame
 * Signature: (J[B)I
 */
JNIEXPORT jint JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_feedFrame
    (JNIEnv *env, jobject obj, jlong tracker_pointer, jbyteArray j_frame) {
  ObjTracking *tracker = (ObjTracking *) tracker_pointer;
  jbyte *frame = env->GetByteArrayElements(j_frame, NULL);
  int res = tracker->doTrack((const unsigned char *) (frame));
  env->ReleaseByteArrayElements(j_frame, frame, 0);
  if (!res) {
    return -1;
  }
  fflush(stdout), fflush(stderr);
  return 0;
}

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    getTargets
 * Signature: (J)[Lorg/cripac/isee/alg/pedestrian/tracking/Tracklet;
 */
JNIEXPORT jobjectArray JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_getTargets
    (JNIEnv *env, jobject obj, jlong pointer) {
  // Analyze Trajectory class in Java.
  jclass tracklet_class = env->FindClass("org/cripac/isee/alg/pedestrian/tracking/Tracklet");
  if (tracklet_class == NULL) {
    fprintf(stderr, "Error: Cannot find Java class: org/cripac/isee/alg/pedestrian/tracking/Tracklet");
    fflush(stdout), fflush(stderr);
    return NULL;
  }
  jmethodID tracklet_constructor = env->GetMethodID(tracklet_class, "<init>", "()V");
  jfieldID loc_seq_field =
      env->GetFieldID(tracklet_class,
                      "locationSequence",
                      "[Lorg/cripac/isee/alg/pedestrian/tracking/Tracklet$BoundingBox;");
  jfieldID start_frame_idxfield =
      env->GetFieldID(tracklet_class,
                      "startFrameIndex",
                      "I");

  // Analyze BoundingBox class in Java.
  jclass bbox_class = env->FindClass("org/cripac/isee/alg/pedestrian/tracking/Tracklet$BoundingBox");
  if (bbox_class == NULL) {
    fprintf(stderr, "Error:Can't find Class:org/cripac/isee/alg/pedestrian/tracking/Tracklet$BoundingBox");
    fflush(stdout), fflush(stderr);
    return NULL;
  }
  jmethodID bbox_constructor = env->GetMethodID(bbox_class, "<init>", "()V");
  jfieldID bbox_x_field = env->GetFieldID(bbox_class, "x", "I");
  jfieldID bbox_y_field = env->GetFieldID(bbox_class, "y", "I");
  jfieldID bbox_width_field = env->GetFieldID(bbox_class, "width", "I");
  jfieldID bbox_height_field = env->GetFieldID(bbox_class, "height", "I");
  jfieldID bbox_patch_data_field = env->GetFieldID(bbox_class, "patchData", "[B");

  // Get tracking results.
  ObjTracking *tracker = (ObjTracking *) pointer;
  int num_tracklets = 0;
  Trajectory *tracklets = tracker->getTrajs(num_tracklets);

  // Create Java tracklet array with equal length with local tracklet result list.
  jobjectArray j_tracklets = env->NewObjectArray((jsize) num_tracklets, tracklet_class, NULL);
  // Fill data of native tracklets into the Java array one by one.
  for (int i = 0; i < num_tracklets; ++i) {
    // For current tracklet.
    const Trajectory &tracklet = tracklets[i];
    // Create a Java tracklet.
    jobject j_tracklet = env->NewObject(tracklet_class, tracklet_constructor);

    // Create Java bounding box array with equal length with the length of current tracklet.
    jobjectArray j_bboxes = env->NewObjectArray(tracklet.traj_size, bbox_class, NULL);
    // Fill data of native bounding box into the Java array one by one.
    for (int j = 0; j < tracklet.traj_size; ++j) {
      // Create a Java bounding box.
      jobject j_bbox = env->NewObject(bbox_class, bbox_constructor);
      // Fill the x, y, width, height fields of the bounding box.
      env->SetIntField(j_bbox, bbox_x_field, tracklet.location_sequence[j].x);
      env->SetIntField(j_bbox, bbox_y_field, tracklet.location_sequence[j].y);
      env->SetIntField(j_bbox, bbox_width_field, tracklet.location_sequence[j].width);
      env->SetIntField(j_bbox, bbox_height_field, tracklet.location_sequence[j].height);

      // Fill the patch data field of the bounding box.
      jbyteArray j_patch_data =
          env->NewByteArray(tracklet.location_sequence[j].width * tracklet.location_sequence[j].height * 3);
      env->SetByteArrayRegion(j_patch_data,
                              0,
                              tracklet.location_sequence[j].width * tracklet.location_sequence[j].height * 3,
                              (jbyte *) tracklet.location_sequence[j].patch_data);
      env->SetObjectField(j_bbox, bbox_patch_data_field, j_patch_data);

      // Insert the new bounding box into the Java bounding box array.
      env->SetObjectArrayElement(j_bboxes, j, j_bbox);

      env->DeleteLocalRef(j_patch_data);
      env->DeleteLocalRef(j_bbox);
    }
    // Put the bounding box array to the locationSequence field of the Java tracklet.
    env->SetObjectField(j_tracklet, loc_seq_field, j_bboxes);
    // Set the startFrameIndex field of the Java tracklet.
    env->SetIntField(j_tracklet, start_frame_idxfield, tracklet.start_frame_idx);
    // Insert the new tracklet into the Java tracklet array.
    env->SetObjectArrayElement(j_tracklets, i, j_tracklet);

    env->DeleteLocalRef(j_tracklet);
    env->DeleteLocalRef(j_bboxes);
  }
  fflush(stdout), fflush(stderr);
  return j_tracklets;
}

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    free
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_free
    (JNIEnv *env, jobject obj, jlong p) {
  delete (ObjTracking *)p;
  fflush(stdout), fflush(stderr);
}
