/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <vector>
#include <points_image.hpp>
#include <stdint.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <iostream>

static cv::Mat invRt, invTt;
static bool init_matrix = false;

void resetMatrix()
{
  init_matrix = false;
}

void initMatrix(const cv::Mat &cameraExtrinsicMat)
{
  invRt = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
  cv::Mat invT = -invRt.t() * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));
  invTt = invT.t();
  init_matrix = true;
}

sensor_msgs::ImagePtr pointcloud2_to_image(cv::Mat image,
                                           const sensor_msgs::PointCloud2ConstPtr &pointcloud2,
                                           const cv::Mat &cameraExtrinsicMat, const cv::Mat &cameraMat,
                                           const cv::Mat &distCoeff, const cv::Size &imageSize)
{
  int w = imageSize.width;
  int h = imageSize.height;
  double *distance = new double[w * h * sizeof(double)];

  int drawn_size = 4;
  cv::Mat color_map_;
  // set color map
  cv::Mat gray_scale(256, 1, CV_8UC1);

  for (int i = 0; i < 256; i++) {
    gray_scale.at<uchar>(i) = i;
  }

  cv::applyColorMap(gray_scale, color_map_, cv::COLORMAP_JET);
  // cv::Mat image(h, w, CV_8UC3, cv::Scalar(0));

  std::cout << image.size() << std::endl;

  uintptr_t cp = (uintptr_t)pointcloud2->data.data();

  if (!init_matrix)
  {
    initMatrix(cameraExtrinsicMat);
  }
  cv::Mat point(1, 3, CV_64F);
  cv::Point2d imagepoint;

  //get max_distance

  float min_distance, max_distance;
  min_distance = max_distance = 0;

  for (uint32_t y = 0; y < pointcloud2->height; ++y)
  {
    for (uint32_t x = 0; x < pointcloud2->width; ++x)
    {
      float *fp = (float *)(cp + (x + y * pointcloud2->width) * pointcloud2->point_step);
   

      for (int i = 0; i < 3; i++)
      {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
          point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
      }

      if (point.at<double>(2) <= 1 )
      {
        continue;
      }

        float distance = point.at<double>(2);
        max_distance = (distance > max_distance) ? distance : max_distance;
        min_distance = (distance < min_distance) ? distance : min_distance;
 
    }
  }

  float distance_range = max_distance - min_distance;

  // printf("max_d: %.2f, min_d: %.2f", max_distance, min_distance);

  for (uint32_t y = 0; y < pointcloud2->height; ++y)
  {
    for (uint32_t x = 0; x < pointcloud2->width; ++x)
    {
      float *fp = (float *)(cp + (x + y * pointcloud2->width) * pointcloud2->point_step);
      double intensity = fp[4];

      for (int i = 0; i < 3; i++)
      {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
          point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
      }

      if (point.at<double>(2) <= 1)
      {
        continue;
      }

      double tmpx = point.at<double>(0) / point.at<double>(2);
      double tmpy = point.at<double>(1) / point.at<double>(2);
      double r2 = tmpx * tmpx + tmpy * tmpy;
      double tmpdist =
          1 + distCoeff.at<double>(0) * r2 + distCoeff.at<double>(1) * r2 * r2 + distCoeff.at<double>(4) * r2 * r2 * r2;

      imagepoint.x =
          tmpx * tmpdist + 2 * distCoeff.at<double>(2) * tmpx * tmpy + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
      imagepoint.y =
          tmpy * tmpdist + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * distCoeff.at<double>(3) * tmpx * tmpy;
      imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
      imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);

      int px = int(imagepoint.x + 0.5);
      int py = int(imagepoint.y + 0.5);

      if (0 <= px && px < w && 0 <= py && py < h)
      {
        int pid = py * w + px;
        if (distance[pid] == 0 || distance[pid] > point.at<double>(2))
        {
          if (point.at<double>(2) >= 0)
          {
                    // Draw a point
            int minus_offset = 0;
            int plus_offset = static_cast<int>(drawn_size/2);
            if (drawn_size % 2 == 0) {
              minus_offset = static_cast<int>(drawn_size/2) - 1;
            } else {
              minus_offset = static_cast<int>(drawn_size/2);
            }
            // Specify which color will be use for this point
            int color_id = distance_range ? ((point.at<double>(2) - min_distance) * 255 / distance_range) : 128;

            // Divide color into each element
            cv::Vec3b color = color_map_.at<cv::Vec3b>(color_id);
            int red   = color[0];
            int green = color[1];
            int blue  = color[2];

            cv::rectangle(image,
                          cv::Point(px - minus_offset, py - minus_offset),
                          cv::Point(px + plus_offset, py + plus_offset),
                          CV_RGB(red, green, blue),
                          CV_FILLED);
            // std::cout << "px: " << px << " py: " << py << " red: " << red << std::endl;
            // image.at<cv::Vec3b>(py, px)[0] = int(point.at<double>(2) * 255 / 150);
            // image.at<cv::Vec3b>(py, px)[0] = int(point.at<double>(2) * 255 / 100);
            // image.at<cv::Vec3b>(py, px)[1] = 0;
            // image.at<cv::Vec3b>(py, px)[2] = 255;
          }
        }
      }
    }
  }
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  delete[] distance;
  return msg;
}

/*autoware_msgs::CameraExtrinsic
pointcloud2_to_3d_calibration(const sensor_msgs::PointCloud2ConstPtr& pointcloud2,
            const cv::Mat& cameraExtrinsicMat)
{
  autoware_msgs::CameraExtrinsic msg;
  std::vector<double> cali;
  for (int y = 0; y < msg.ysize ; ++y) {
    for (int x = 0; x < msg.xsize ; ++x){
      cali.push_back(cameraExtrinsicMat.at<double>(y,x));
    }
  }

  msg.calibration = cali;
  return msg;
}
 */
