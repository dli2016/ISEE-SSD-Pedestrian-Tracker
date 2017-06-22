//
// Created by ken.yu on 16-10-6.
//

#include <cerrno>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <linux/limits.h>

#include <opencv2/opencv.hpp>
#include <fstream>

#include "ObjTracking.hpp"

#define BUF_SIZE 10000

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    printf("Usage: pedestrian_tracker_test path/to/config/file path/to/test/video/list\n");
    return 1;
  }
  printf("Called with %s %s\n", argv[1], argv[2]);

  printf("Loading video list %s...\n", argv[2]);
  FILE *video_list = fopen(argv[2], "r");
  if (video_list == NULL) {
    fprintf(stderr, "Cannot open video list!\n");
    return -1;
  }

  FILE *conf = fopen(argv[1], "r");
  char *buf = (char *) malloc(BUF_SIZE);
  int len = fread(buf, 1, BUF_SIZE, conf);
  printf("Read config file with len=%d\n", len);
  fclose(conf);

  while (!feof(video_list)) {
    char video_path[PATH_MAX];
    fgets(video_path, PATH_MAX, video_list);
    for (int i = strlen(video_path) - 1; i >= 0 && video_path[i] == '\n'; --i)
      video_path[i] = '\0';
    if (strlen(video_path) == 0)
      continue;

    fprintf(stdout, "Reading video %s...\n", video_path);
    VideoCapture cap(video_path);

    Mat frame;
    cap >> frame;
    if (frame.empty()) {
      fprintf(stderr, "Cannot read video %d!\n", video_path);
      return 2;
    }
    fprintf(stdout, "Video opened!\n");

    ObjTracking tracker;

    ObjTrackingParams params;
    params.w = frame.cols;
    params.h = frame.rows;
    params.chns_num = frame.channels();
    params.proto_path = "models/SSDCaffe/deploy.prototxt";
    params.weights_path = "models/SSDCaffe/deploy.caffemodel";
    params.buffer = buf;
    params.buffer_len = len;
    params.gpu_index = 0;
    params.confidence_threshold = 0.5f;
    params.detection_method = SSD;

    tracker.init(params);
    fprintf(stdout, "Tracker initialized!\n");

    int start = 3500, end = 4200;
    int cnt = 0;
    while (!frame.empty()) {
      ++cnt;
//      imshow("Frame", frame);
//      waitKey(1);
      if (cnt > start) {
        printf("%d\n", cnt);
        tracker.doTrack(frame.data);
      }
      if (cnt > end)
        break;
      cap >> frame;
    }
    fprintf(stdout, "Tracked on %d frames!\n", cnt);
    cap.release();

    int num_tracklets;
    Trajectory *tracklets = tracker.getTrajs(num_tracklets);
    printf("Tracked %d pedestrians!\n", num_tracklets);
    for (int i = 0; i < num_tracklets; ++i) {
      tracklets[i].start_frame_idx += start,
      printf("\t%d -> %d\n",
             tracklets[i].start_frame_idx,
             tracklets[i].start_frame_idx + tracklets[i].traj_size);
    }

    // Save the tracklets to a result file.
    string name = video_path;
    name = name.substr(0, name.find('.')) + ".txt";
    ofstream fout(name);
    fout << num_tracklets << endl << endl;
    for (int i = 0; i < num_tracklets; ++i) {
      const Trajectory &tracklet = tracklets[i];
      fout << tracklet.start_frame_idx << endl;
      fout << tracklet.traj_size << endl;
      for (int j = 0; j < tracklet.traj_size; ++j) {
        const BoundingBox &bbox = tracklet.location_sequence[j];
        fout << bbox.x << ' ' << bbox.y << ' ' << bbox.width << ' ' << bbox.height << endl;
      }
      fout << endl;
    }
    fout.close();
    printf("Results saved!\n");

    // Display
    const char *windowName = "result";
    cap.open(video_path);
    int interval = 900 / cap.get(CV_CAP_PROP_FPS);
    int frame_idx = 0;
    while (true) {
      Mat frame;
      cap >> frame;
      ++frame_idx;
      if (frame.empty())
        continue;
      if (frame_idx <= start) {
//        printf("%d\n", frame_idx);
        continue;
      }
      cv::putText(frame, to_string(frame_idx), Point(200, 200), CV_FONT_BLACK, 3.0, Scalar(255, 0, 0));
      for (int i = 0; i < num_tracklets; ++i)
        if (tracklets[i].start_frame_idx <= frame_idx
            && tracklets[i].start_frame_idx + tracklets[i].traj_size > frame_idx) {
          const BoundingBox &bbox = tracklets[i].location_sequence[frame_idx - tracklets[i].start_frame_idx];
          rectangle(frame, Rect(bbox.x, bbox.y, bbox.width, bbox.height), Scalar(255, 0, 0));
        }
      imshow(windowName, frame);
      waitKey(1);
      int key_pressed = waitKey(interval);
      if ((key_pressed & ((1 << 8) - 1)) == ' ') {
        destroyWindow(windowName);
        break;
      }
    }
    destroyWindow(windowName);
    cap.release();
  }
  free(buf);
  fclose(video_list);

  return 0;
}
