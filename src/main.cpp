#include "myUtil.h"

using namespace cv;
using namespace std;
using namespace Eigen;

double add_angle(double a, double b) {
  if ( a+b > MY_PI) return a+b-MY_PI*2.0;
  else return a+b;
}

int main( int argc, char** argv )	{
  if (argc < 2) {
    cout << "[usage] filename" << endl;
    return 0;
  }

  // 画像読み込み
  string filename = argv[1];
  Mat img_big = imread(filename, 0);

  // リサイズ
  Mat img;
  resize(img_big, img, Size(0,0), 0.15, 0.15);

  // 画像サイズ取得，画像中心を保存
  int height = img.rows;
  int width = img.cols;
  cout << "[DEBUG] width, height = " << width << "," << height << endl;
  int center_row = height/2;
  int center_col = width/2;
  //cout << center_row << "," << center_col << endl;
  Point2f center(center_col, center_row);

  Mat img_cp_bottom = img.clone();
  Mat img_cp_top = img.clone();

  for(int u = 0; u < width; u++) {
    for(int v = 0; v < height; v++) {
      double theta = (u-center_col)*2.0*MY_PI/width;
      double phi = (v-center_row)*MY_PI/height;
      Vector3d vec;
      vec << cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
      Matrix3d R;
      double alpha = MY_PI/3.0;
      R << 1.0, 0, 0,
           0, cos(alpha), -sin(alpha),
           0, sin(alpha), cos(alpha);
      Vector3d vec_rotated = R*vec;
      phi = asin(vec_rotated[1]);
      if(vec_rotated[0] > 0 && vec_rotated[2] > 0) {
        theta = asin(vec_rotated[0]/cos(phi));
      } else if (vec_rotated[0] > 0 && vec_rotated[2] < 0) {
        theta = acos(vec_rotated[2]/cos(phi));
      } else if (vec_rotated[0] < 0 && vec_rotated[2] > 0) {
        theta = asin(vec_rotated[0]/cos(phi));
      } else if (vec_rotated[0] < 0 && vec_rotated[2] < 0) {
        theta = -1.0*acos(vec_rotated[2]/cos(phi));
      }
      int rotate_u = theta*width/(2.0*MY_PI)+center_col;
      int rotate_v = phi*height/MY_PI+center_row;
      if (rotate_u < 0 || rotate_u >= width) continue;
      if (rotate_v < 0 || rotate_v >= height) continue;
      img_cp_top.at<unsigned char>(v, u) = img.at<unsigned char>(rotate_v, rotate_u);
    }
  }

  for(int u = 0; u < width; u++) {
    for(int v = 0; v < height; v++) {
      double theta = (u-center_col)*2.0*MY_PI/width;
      double phi = (v-center_row)*MY_PI/height;
      Vector3d vec;
      vec << cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
      Matrix3d R;
      double alpha = -1.0*MY_PI/3.0;
      R << 1.0, 0, 0,
           0, cos(alpha), -sin(alpha),
           0, sin(alpha), cos(alpha);
      Vector3d vec_rotated = R*vec;
      phi = asin(vec_rotated[1]);
      if(vec_rotated[0] > 0 && vec_rotated[2] > 0) {
        theta = asin(vec_rotated[0]/cos(phi));
      } else if (vec_rotated[0] > 0 && vec_rotated[2] < 0) {
        theta = acos(vec_rotated[2]/cos(phi));
      } else if (vec_rotated[0] < 0 && vec_rotated[2] > 0) {
        theta = asin(vec_rotated[0]/cos(phi));
      } else if (vec_rotated[0] < 0 && vec_rotated[2] < 0) {
        theta = -1.0*acos(vec_rotated[2]/cos(phi));
      }
      int rotate_u = theta*width/(2.0*MY_PI)+center_col;
      int rotate_v = phi*height/MY_PI+center_row;
      if (rotate_u < 0 || rotate_u >= width) continue;
      if (rotate_v < 0 || rotate_v >= height) continue;
      img_cp_bottom.at<unsigned char>(v, u) = img.at<unsigned char>(rotate_v, rotate_u);
    }
  }
/*
  for(int u = 0; u < width; u++) {
    for(int v = 0; v < height; v++) {
      double theta = (u-center_col)*2.0*MY_PI/width;
      double phi = (v-center_row)*MY_PI/height;
      //cout << "[DEBUG] " << RAD2DEG(theta) << ", " << RAD2DEG(phi) << endl;
      if ( theta > -1.0*MY_PI/2.0 && theta < MY_PI/2.0) {
        phi += MY_PI/3.0;
        if(phi > MY_PI/2.0) {
          phi = MY_PI - phi;
          theta = add_angle(theta, MY_PI);
        }
      } else {
         phi -= MY_PI/3.0;
        if(phi < -1.0*MY_PI/2.0) {
          phi = -1.0*(MY_PI + phi);
          theta = add_angle(theta, MY_PI);
        }
      }
      int rotate_u = theta*width/(2.0*MY_PI)+center_col;
      int rotate_v = phi*height/MY_PI+center_row;
      if (rotate_u < 0 || rotate_u >= width) continue;
      if (rotate_v < 0 || rotate_v >= height) continue;
      img_cp_bottom.at<unsigned char>(v, u) = img.at<unsigned char>(rotate_v, rotate_u);
    }
  }
*/
/*
  for(int u = 0; u < width; u++) {
    for(int v = 0; v < height; v++) {
      double theta = (u-center_col)*2.0*MY_PI/width;
      double phi = (v-center_row)*MY_PI/height;
      //cout << "[DEBUG] " << RAD2DEG(theta) << ", " << RAD2DEG(phi) << endl;
      phi -= MY_PI/3.0;
      if(phi < -1.0*MY_PI/2.0) {
        phi = -1.0*(MY_PI + phi);
        theta = add_angle(theta, MY_PI);
      }
      int rotate_u = theta*width/(2.0*MY_PI)+center_col;
      int rotate_v = phi*height/MY_PI+center_row;
      if (rotate_u < 0 || rotate_u >= width) continue;
      if (rotate_v < 0 || rotate_v >= height) continue;
      img_cp_top.at<unsigned char>(v, u) = img.at<unsigned char>(rotate_v, rotate_u);
    }
  }
*/
  // 回転の結果
  cv::imshow("img", img);
  cv::waitKey();
  cv::imshow("img_cp_bottom", img_cp_bottom);
  cv::waitKey();
  cv::imshow("img_cp_top", img_cp_top);
  cv::waitKey();

  // 特徴点検出(すべて)
  vector<KeyPoint> kpts1, kpts2, kpts3;
  Mat dst1, dst2, dst3;
  Ptr<AKAZE> akaze = AKAZE::create();
  akaze->detect(img, kpts1);
  akaze->detect(img_cp_bottom, kpts2);
  akaze->detect(img_cp_top, kpts3);

  // 指定範囲内のみの特徴点だけ残す
  for(auto itr = kpts1.begin(); itr != kpts1.end(); ++itr) {
    Point2f kp = itr->pt;
    double theta = (kp.x-center_col)*2.0*MY_PI/width;
    double phi = (kp.y-center_row)*MY_PI/height;
    if(atan(tan(phi)/cos(theta)) > MY_PI/6.0 || atan(tan(phi)/cos(theta)) < -1.0*MY_PI/6.0) {
      if (itr != kpts1.end()) {
        kpts1.erase(itr);
        itr--;
      }
    }
  }

  for(auto itr = kpts2.begin(); itr != kpts2.end(); ++itr) {
    Point2f kp = itr->pt;
    double theta = (kp.x-center_col)*2.0*MY_PI/width;
    double phi = (kp.y-center_row)*MY_PI/height;
    if(atan(tan(phi)/cos(theta)) > MY_PI/6.0 || atan(tan(phi)/cos(theta)) < -1.0*MY_PI/6.0) {
      if (itr != kpts2.end()) {
        kpts2.erase(itr);
        itr--;
      }
    }
  }

  for(auto itr = kpts3.begin(); itr != kpts3.end(); ++itr) {
    Point2f kp = itr->pt;
    double theta = (kp.x-center_col)*2.0*MY_PI/width;
    double phi = (kp.y-center_row)*MY_PI/height;
    if(atan(tan(phi)/cos(theta)) > MY_PI/6.0 || atan(tan(phi)/cos(theta)) < -1.0*MY_PI/6.0) {
      if (itr != kpts3.end()) {
        kpts3.erase(itr);
        itr--;
      }
    }
  }

  cv::drawKeypoints(img, kpts1, dst1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(img_cp_bottom, kpts2, dst2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::drawKeypoints(img_cp_top, kpts3, dst3, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow("img1", dst1);
  cv::waitKey();
  cv::imshow("img2", dst2);
  cv::waitKey();
  cv::imshow("img3", dst3);
  cv::waitKey();


  cout << "End." << endl;
  return 0;
}
