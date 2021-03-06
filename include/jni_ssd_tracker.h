/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class org_cripac_isee_alg_pedestrian_tracking_SSDTracker */

#ifndef _Included_org_cripac_isee_alg_pedestrian_tracking_SSDTracker
#define _Included_org_cripac_isee_alg_pedestrian_tracking_SSDTracker
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    initialize
 * Signature: (Lorg/cripac/isee/alg/pedestrian/tracking/SSDTracker/SSDTrackerParams;)J
 */
JNIEXPORT jlong JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_initialize
  (JNIEnv *, jobject, jobject);

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    feedFrame
 * Signature: (J[B)I
 */
JNIEXPORT jint JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_feedFrame
  (JNIEnv *, jobject, jlong, jbyteArray);

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    getTargets
 * Signature: (J)[Lorg/cripac/isee/alg/pedestrian/tracking/Tracklet;
 */
JNIEXPORT jobjectArray JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_getTargets
  (JNIEnv *, jobject, jlong);

/*
 * Class:     org_cripac_isee_alg_pedestrian_tracking_SSDTracker
 * Method:    free
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_org_cripac_isee_alg_pedestrian_tracking_SSDTracker_free
  (JNIEnv *, jobject, jlong);

#ifdef __cplusplus
}
#endif
#endif
