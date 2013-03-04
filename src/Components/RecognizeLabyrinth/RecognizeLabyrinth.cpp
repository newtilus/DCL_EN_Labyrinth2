/*!
 * \file
 * \brief
 * \author Emil Natil
 */


#include "Logger.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <fstream>


#include "RecognizeLabyrinth.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace std;
//using Types::Mrrocpp_Proxy::RecognizeLabyrinth_Reading;

namespace Processors {
namespace RecognizeLabyrinth {

// OpenCV writes hue in range 0..180 instead of 0..360
#define H(x) (x>>1)

bool check_connection(Mat* img, Point from, Point to, int cell_width, int cell_height)
{
  // check how both cells are oriented
  int direction;
  if (abs(from.x - to.x) > abs(from.y - to.y))
    direction = HORIZONTAL;
  else
    direction = VERTICAL;

  Point tmp;
  Mat roi;

  // crop the two cells from the image
  if (direction == HORIZONTAL) {
    if (from.x > to.x) {
      tmp = to;
      to = from;
      from = tmp;
    }
    roi = Mat(*img, Rect(from.x, from.y - cell_height / 2, to.x - from.x, cell_height));
  } else {
    if (from.y > to.y) {
      tmp = to;
      to = from;
      from = tmp;
    }
    roi = Mat(*img, Rect(from.x - cell_width / 2, from.y, cell_width, to.y - from.y));
  }

  // BW, threshold and find lines
  Mat bw;
  cvtColor(roi, bw, CV_BGR2GRAY);
  Mat after_threshold;
  cv::threshold(bw, after_threshold, 220, 255, CV_THRESH_BINARY);
  vector<Vec4i> lines;
  int count_proper_lines = 0;
  if (direction == HORIZONTAL)
    HoughLinesP(after_threshold, lines, 1, CV_PI/2, 12,cell_height*0.4, 1);
  else
    HoughLinesP(after_threshold, lines, 1, CV_PI/2, 12,cell_width*0.4, 1);

  // count the lines of the correct size to threat them as walls
  for (size_t i = 0; i < lines.size(); i++) {
    if (direction == HORIZONTAL) {
      if (abs(lines[i][0] - lines[i][2]) < 2)
        ++count_proper_lines;
    } else {
      if (abs(lines[i][1] - lines[i][3]) < 2)
        ++count_proper_lines;
    }
  }

  // if there are no walls, paint the line and return true
  if (count_proper_lines < 1) {
    line(*img, from, to, CV_RGB(0, 255, 0), 2, CV_AA, 0);
    return true;
  } else
    return false;

}

RecognizeLabyrinth::RecognizeLabyrinth(const std::string & name) :
  Base::Component(name),
  prop_width("width", 3),
  prop_height("height", 3),
  //prop_calibrationResults("calibrationResults", "1"),
  prop_x1("x1", 1),
  prop_x2("x2", 1),
  prop_y1("y1", 1),
  prop_y2("y2", 1),
  prop_threshold("threshold", 20),
  prop_segmentation_threshold("segmentation_threshold", 20),
  prop_min_length("min_length", 100),
  prop_max_gap("max_gap", 5)
{
  printf("\n\nSiema!\n");
  LOG(LNOTICE) << "Hello RecognizeLabyrinth\n";
  registerProperty(prop_width);
  registerProperty(prop_height);
  //registerProperty(prop_calibrationResults);
  registerProperty(prop_x1);
  registerProperty(prop_x2);
  registerProperty(prop_y1);
  registerProperty(prop_y2);
  registerProperty(prop_threshold);
  registerProperty(prop_segmentation_threshold);
  registerProperty(prop_min_length);
  registerProperty(prop_max_gap);
  k = 0;
  showImage = false;
  showProcessed = false;
  solve = false;
  calibrate = false;
}

RecognizeLabyrinth::~RecognizeLabyrinth()
{
  LOG(LNOTICE) << "Good bye RecognizeLabyrinth\n";
}

bool RecognizeLabyrinth::onInit()
{
  LOG(LNOTICE) << "RecognizeLabyrinth::onInit\n";

  file_found = loadParameters();
  labyrinth_found = false;
  labyrinth_solved = false;
  min_x = 0;
  min_y = 0;
  max_x = 0;
  max_y = 0;
  for(int i=0; i<MAX_PATH; ++i) path[i] = 0;

  return true;
}

bool RecognizeLabyrinth::onFinish()
{
  LOG(LNOTICE) << "RecognizeLabyrinth::finish\n";

  return true;
}

bool RecognizeLabyrinth::onStep()
{
  LOG(LNOTICE) << "RecognizeLabyrinth::step\n";
  return true;
}

bool RecognizeLabyrinth::onStop()
{
  return true;
}

bool RecognizeLabyrinth::onStart()
{
  return true;
}

void RecognizeLabyrinth::onRpcCall()
{
  LOG(LNOTICE) << "void RecognizeLabyrinth::onRpcCall() begin\n";
}

void RecognizeLabyrinth::onNewImage()
{
  //LOG(LNOTICE) << "ImageLabyrinth_Processor::onNewImage\n";

  x1 = prop_x1;
  x2 = prop_x2;
  y1 = prop_y1;
  y2 = prop_y2;
  threshold = prop_threshold;
  segmentation_threshold = prop_segmentation_threshold;
  min_length = prop_min_length;
  max_gap = prop_max_gap;

  // Sometimes data is not available, don't care about it and repeat the loop
  if(in_img.empty())
    return;

  Mat image = in_img.read();
//  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


  // Remember the first picture
  if(first_image_saved != true)
  {
    first_image = image;
    first_image_saved = true;
    return;
  }

  // TODO Comment this to continue with labyrinth solving
//  out_img.write(image.clone());
//  return;

  //if(labyrinth_found!=true) { labyrinth_found=true; labyrinth_first = image.clone(); }
  // Extract the manipulator's arm and other objects from the original picture
//  Mat difference;
//  Mat difference_bw;
//  Mat difference_threshold;
//  Mat difference_final;
//  Mat difference_final_inv;
//  absdiff(first_image, image, difference);
//  cvtColor(difference, difference_bw, CV_BGR2GRAY);
//  cv::threshold(difference_bw, difference_threshold, 200, 255, THRESH_TOZERO_INV);
//  cv::threshold(difference_threshold, difference_final, 50, 255, THRESH_BINARY);
//  bitwise_not(difference_final, difference_final_inv);
  // TODO Delete this comment to enable above extraction
//  if(labyrinth_found)
//  labyrinth_first.copyTo(image, difference_final);


  Mat out_calib;
  // Calibration based on file, if file doesn't exist, don't calibrate
//  if (file_found) {
//    Mat map1, map2;
//    Size imageSize = image.size();
//    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_32FC1, map1, map2);
//    remap(image, out_calib, map1, map2, INTER_LINEAR);
//    image = out_calib;
//  }


  int first_image_width = image.size().width;
  int first_image_height = image.size().height;
  Mat rotatedImg;
  Mat labyrinth_after_first;
  Mat src;
  Mat src_bw;
  Mat after_threshold;
  Mat src_lines;


  // if the labyrinth was not found, rotate image and try to find walls, remember the rotation
  if (!labyrinth_found)
  {
    out_img.write(src.clone());

    int line_number_left = 0;
    int line_number_right = 0;
    int line_number_up = 0;
    int line_number_down = 0;
    int min_line_length = 50;

    for (rotation = 1.0; rotation < 180.0; rotation += 1) {
      //LOG(LNOTICE) << " rotation " << rotation;

      Mat image_bigger(image.rows + image.rows, image.cols + image.cols, image.depth());
      copyMakeBorder(image, image_bigger, (image.cols)/2, (image.cols)/2, (image.rows)/2, (image.rows)/2, BORDER_REPLICATE);

      Point2f src_center(image_bigger.cols/2.0F, image_bigger.rows/2.0F);
      Mat rot_mat = getRotationMatrix2D(src_center, rotation, 1.0);

      warpAffine(image_bigger, rotatedImg, rot_mat, image_bigger.size());

      src = rotatedImg.clone();

      CvSize src_image_size = src.size();

      // Source bw image
      cvtColor(src, src_bw, CV_BGR2GRAY);

      // Threshold
      after_threshold = cvCreateImage(src_image_size, IPL_DEPTH_8U, 1);
      cv::threshold(src_bw, after_threshold, segmentation_threshold, 255, CV_THRESH_BINARY);
      //src_bw = src_bw > segmentation_threshold;     // fajny efekt

      vector<Vec4i> lines;
      src_lines = src.clone();

      HoughLinesP(after_threshold, lines, 1, CV_PI/2, threshold, min_length, max_gap);

      if (lines.size() == 0)
        continue;

      // Draw lines
//      for (size_t i = 0; i < lines.size(); i++) {
//            line(src_lines, Point(lines[i][0], lines[i][1]),  Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
//      }

      min_x = 0;
      max_x = 0;
      min_y = 0;
      max_y = 0;
      line_number_left = 0;
      line_number_right = 0;
      line_number_up = 0;
      line_number_down = 0;
      for (size_t i = 0; i < lines.size(); i++) {

        int x0 = lines[i][0];
        int y0 = lines[i][1];
        int x1 = lines[i][2];
        int y1 = lines[i][3];

        // vertical lines
        // calculate the coordinates of the middle of the line
        int middle = (x0 + x1) / 2;

        // the first line on the left?
        if ((middle < min_x || min_x == 0) && abs(y0 - y1) > min_line_length) {
          min_x = middle;
          line_number_left = i;
        }

        // the first line on the right?
        if (middle > max_x && abs(y0 - y1) > min_line_length) {
          max_x = middle;
          line_number_right = i;
        }

        // horizontal lines
        // calculate the coordinates of the middle of the line
        middle = (y0 + y1) / 2;
        // the first line up?
        if ((middle < min_y || min_y == 0) && abs(x0 - x1) > min_line_length) {
          min_y = middle;
          line_number_up = i;
        }

        // the first line down?
        if (middle > max_y && abs(x0 - x1) > min_line_length) {
          max_y = middle;
          line_number_down = i;
        }

      }


      // if the walls were found and the labyrinth is right size
      if (abs(max_x-min_x) > first_image_width / 3 && abs(max_y-min_y) > first_image_height / 3) {
        //crop the image of labyrinth
        Mat roi(src_lines, Rect(min_x,min_y,max_x-min_x,max_y-min_y));
        labyrinth_first = roi.clone();
        labyrinth_after_first = roi.clone();

  //      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,line_number_left);
  //      cvLine(src_lines, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), cvScalar(0,255,0), 4);
  //      line = (CvPoint*)cvGetSeqElem(lines,line_number_right);
  //      cvLine(src_lines, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), cvScalar(0,255,0), 4);
  //      line = (CvPoint*)cvGetSeqElem(lines,line_number_up);
  //      cvLine(src_lines, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), cvScalar(0,255,0), 4);
  //      line = (CvPoint*)cvGetSeqElem(lines,line_number_down);
  //      cvLine(src_lines, cvPoint(line[0].x, line[0].y), cvPoint(line[1].x, line[1].y), cvScalar(0,255,0), 4);

        LOG(LNOTICE) << "Labyrinth found.";
        LOG(LNOTICE) << " min_x " << min_x << " min_y " << min_y << " max_x " << max_x << " max_y " << max_y << " rotation " << rotation;
        labyrinth_found = true;
        break;
      }

    }
  }
  // No labyrinth found in the picture
  else if (min_x == 0 && min_y == 0 && max_x == 0 && max_y == 0)
  {
    LOG(LWARNING) << "Labyrinth couldn't be found.";
    return;
  }
  // Labyrinth found before, rotate and crop the actual image regarding to the first image of labyrinth
  else
  {
    //LOG(LNOTICE) << "Znalazlem już wcześniej labirynt i go pamiętam. Przycinam i obracam nowy obraz wg obliczeń z pierwszego.";

    // TODO Optimize, remember the values in class params instead of calculate for every frame
    Mat image_bigger(image.rows + image.rows, image.cols + image.cols, image.depth());

    copyMakeBorder(image, image_bigger, (image.cols)/2, (image.cols)/2, (image.rows)/2, (image.rows)/2, BORDER_REPLICATE);

    Point2f src_center(image_bigger.cols/2.0F, image_bigger.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, rotation, 1.0);

    warpAffine(image_bigger, rotatedImg, rot_mat, image_bigger.size());

    src = rotatedImg.clone();
    Mat roi(src, Rect(min_x,min_y,max_x-min_x,max_y-min_y));
    // switch between these two to solve labyrinth online or just once, at the beginning
    //labyrinth_after_first = roi.clone();
    labyrinth_after_first = labyrinth_first;
  }

  Mat labyrinth_cropped = labyrinth_after_first.clone();


  Labyrinth labyrinth;

  int cell_width = labyrinth_cropped.size().width / DIMENSION_X;
  int cell_height = labyrinth_cropped.size().height / DIMENSION_Y;

  for (int i = 0; i < DIMENSION_X; i++) {
    for (int j = 0; j < DIMENSION_Y; j++) {
      //bool insert_cell(int pos_x, int pos_y, int x, int y, int width, int heigth);
      labyrinth.insert_cell(i, j, labyrinth_cropped.size().width / DIMENSION_X * i + cell_width / 2, labyrinth_cropped.size().height
          / DIMENSION_Y * j + cell_height / 2, cell_width, cell_height);
      circle(labyrinth_cropped, cvPoint(labyrinth_cropped.size().width / DIMENSION_X * i + cell_width / 2, labyrinth_cropped.size().height
          / DIMENSION_Y * j + cell_height / 2), 2, cvScalar(255, 255, 0), 2, 8, 0);
    }
  }

  for (int j = 0; j < DIMENSION_Y; ++j) {
    for (int i = 0; i < DIMENSION_X - 1; ++i) {
      bool connection_exists =
              check_connection(&labyrinth_cropped, Point(labyrinth_cropped.size().width / DIMENSION_X * i
                  + cell_width / 2, labyrinth_cropped.size().height / DIMENSION_Y * j + cell_height / 2), Point(labyrinth_cropped.size().width
                  / DIMENSION_X * (i + 1) + cell_width / 2, labyrinth_cropped.size().height / DIMENSION_Y
                  * j + cell_height / 2), cell_width, cell_height);
      if (connection_exists) {
        labyrinth.set_transition(i, j, RIGHT);
        labyrinth.set_transition(i + 1, j, LEFT);
      }
    }
  }

  for (int j = 0; j < DIMENSION_X; ++j) {
    for (int i = 0; i < DIMENSION_Y - 1; ++i) {
      bool connection_exists =
              check_connection(&labyrinth_cropped, Point(labyrinth_cropped.size().width / DIMENSION_X * j
                  + cell_width / 2, labyrinth_cropped.size().height / DIMENSION_Y * i + cell_height / 2), Point(labyrinth_cropped.size().width
                  / DIMENSION_X * j + cell_width / 2, labyrinth_cropped.size().height / DIMENSION_Y * (i
                  + 1) + cell_height / 2), cell_width, cell_height);
      if (connection_exists) {
        labyrinth.set_transition(j, i, DOWN);
        labyrinth.set_transition(j, i + 1, UP);
      }
    }
  }

  vector <pair <int, int> > v = labyrinth.solve(make_pair(START_X, START_Y), make_pair(END_X, END_Y));
  vector <pair <int, int> >::iterator it1;
  vector <pair <int, int> >::iterator it2;


  if (v.size() > 1)
    labyrinth_solved = true;
  else
    labyrinth_solved = false;


  if(labyrinth_solved)
  {
    path_size = v.size() - 1; // -1 because v represents each point in path, but counting moves we shouldn't take first point into consideration

    int counter;
    for (it1 = v.begin(), it2 = ++(v.begin()), counter = 0; it2 != v.end() && it1 != v.end(); ++it1, ++it2, ++counter)
    {
      if((*it2).first > (*it1).first) // RIGHT
        path[counter] = RIGHT;
      else if((*it2).first < (*it1).first) // LEFT
        path[counter] = LEFT;
      else if((*it2).second > (*it1).second) // UP
        path[counter] = DOWN;
      else if((*it2).second < (*it1).second) // DOWN
        path[counter] = UP;

      Point from, to;
      from = Point((labyrinth.get_cell((*it1).first, (*it1).second))->x, (labyrinth.get_cell((*it1).first, (*it1).second))->y);
      to = Point((labyrinth.get_cell((*it2).first, (*it2).second))->x, (labyrinth.get_cell((*it2).first, (*it2).second))->y);
      line(labyrinth_cropped, from, to, CV_RGB(255, 255, 0), 2, CV_AA, 0);
    }
    x1 = (*(v.begin())).first;
    y1 = (*(v.begin())).second;
    x2 = (v.back()).first;
    y2 = (v.back()).second;


    // Data to be sent to communication component
    string stringData = "";


    ostringstream ss;
    ss << " " << path_size;
    ss << " " << x1;
    ss << " " << y1;
    ss << " " << x2;
    ss << " " << y2;

    //printf("Solved: %d ", labyrinth_solved);
    //printf("Path Size: %i ", path_size);
    //printf("Start_pt: (%i, %i) ", x1, y1);
    //printf("End_pt: (%i, %i) ", x2, y2);
    //printf("Path: ");
    for(int i=0; i<path_size; ++i) {
      //printf("%i ", path[i]);
      ss << " " << path[i];
    }
    //printf("\n");

    stringData = ss.str();

    out_reading.write(stringData);
  }
  else
  {
    x1 = x2 = y1 = y2 = 0;
    LOG(LNOTICE) << "Couldn't find the correct solution for given labyrinth!";
  }
//  printf("\n\nRecognizeLabyrinth onRpcCall():\n");
//  printf("Solved: %d\n", labyrinth_solved);
//  printf("Path Size: %i\n", path_size);
//  printf("Start_pt: (%i, %i)\n", x1, y1);
//  printf("End_pt: (%i, %i)\n", x2, y2);
//  printf("Path: ");
//  for(int i=0; i<path_size; ++i)
//    printf("%i ", path[i]);

  out_img.write(labyrinth_cropped.clone());

  //labyrinth_found = false;
}

bool RecognizeLabyrinth::loadParameters()
{
  LOG(LNOTICE) << "RecognizeLabyrinth::loadParameters()";

  return false;
  // TODO odczytac z properties stringa z nazw pliku, a stamtad calibrationResults
  /*
   FileStorage fs(props.calibrationResults, FileStorage::READ);
   if (!fs.isOpened()) {
    LOG(LNOTICE) << "Coudn't open file " << props.calibrationResults << " with calibration parameters.";
   return false;
   } else {
     fs["camera_matrix"] >> cameraMatrix;
     fs["distortion_coefficients"] >> distCoeffs;

    LOG(LNOTICE) << "Loaded camera matrix";
    LOG(LNOTICE) << cameraMatrix.at <double> (0, 0) << " " << cameraMatrix.at <double> (0, 1) << " "
        << cameraMatrix.at <double> (0, 2);
    LOG(LNOTICE) << cameraMatrix.at <double> (1, 0) << " " << cameraMatrix.at <double> (1, 1) << " "
        << cameraMatrix.at <double> (1, 2);
    LOG(LNOTICE) << cameraMatrix.at <double> (2, 0) << " " << cameraMatrix.at <double> (2, 1) << " "
        << cameraMatrix.at <double> (2, 2);
    LOG(LNOTICE) << "Loaded distortion coefficients";
    LOG(LNOTICE) << distCoeffs.at <double> (0, 0) << " " << distCoeffs.at <double> (1, 0) << " "
        << distCoeffs.at <double> (2, 0) << " " << distCoeffs.at <double> (3, 0) << " " << distCoeffs.at <
        double> (4, 0) << " " << distCoeffs.at <double> (5, 0) << " " << distCoeffs.at <double> (6, 0) << " "
        << distCoeffs.at <double> (7, 0);
    }
  return true;
  */
}

void RecognizeLabyrinth::prepareInterface() {


  h_onNewImage.setup(this, &RecognizeLabyrinth::onNewImage);

  registerHandler("onNewImage", &h_onNewImage);
  addDependency("onNewImage", &in_img);

  registerStream("in_img", &in_img);
  registerStream("out_img", &out_img);
  registerStream("out_reading", &out_reading);
}


Labyrinth::Labyrinth()
{
  for (int i = 0; i < DIMENSION_X; ++i)
    for (int j = 0; j < DIMENSION_Y; ++j)
      for (int k = 0; k < 4; ++k)
        transitions[i][j][k] = false;

  for (int i = 0; i < DIMENSION_X; ++i)
    for (int j = 0; j < DIMENSION_Y; ++j)
      cells[i][j] = NULL;

  for (int i = 0; i < DIMENSION_X; ++i)
    for (int j = 0; j < DIMENSION_Y; ++j)
      waiting[i][j] = false;

}
;
bool Labyrinth::insert_cell(int pos_x, int pos_y, int x, int y, int width, int height)
{
  cells[pos_x][pos_y] = new Cell(x, y, width, height);

  return true;
}
;

Cell* Labyrinth::get_cell(int pos_x, int pos_y)
{
  return cells[pos_x][pos_y];
}
;

void Labyrinth::set_transition(int pos_x, int pos_y, int direction)
{
  transitions[pos_x][pos_y][direction] = true;
}
;

void Labyrinth::remove_transition(int pos_x, int pos_y, int direction)
{
  transitions[pos_x][pos_y][direction] = false;
}
;

bool Labyrinth::get_transition(int pos_x, int pos_y, int direction)
{
  return transitions[pos_x][pos_y][direction];
}
;

bool Labyrinth::get_waiting(int pos_x, int pos_y)
{
  return waiting[pos_x][pos_y];
}
;

bool Labyrinth::set_waiting(int pos_x, int pos_y)
{
  return waiting[pos_x][pos_y] = true;
}
;

vector <pair <int, int> > Labyrinth::solve(pair <int, int> start, pair <int, int> end)
{
  query.push_back(start);
  path[start.first][start.second].push_back(make_pair(start.first, start.second));

  ofstream file;
  file.open("/home/enatil/solve_maze.txt");

  file << "Transitions:" << endl << "UP" << endl;
  for (int j = 0; j < DIMENSION_Y; ++j) {
    for (int i = 0; i < DIMENSION_X; ++i) {
      file << transitions[i][j][UP] << "\t";
    }
    file << endl;
  }

  file << "RIGHT" << endl;
  for (int j = 0; j < DIMENSION_Y; ++j) {
    for (int i = 0; i < DIMENSION_X; ++i) {
      file << transitions[i][j][RIGHT] << "\t";
    }
    file << endl;
  }

  file << "DOWN" << endl;
  for (int j = 0; j < DIMENSION_Y; ++j) {
    for (int i = 0; i < DIMENSION_X; ++i) {
      file << transitions[i][j][DOWN] << "\t";
    }
    file << endl;
  }

  file << "LEFT" << endl;
  for (int j = 0; j < DIMENSION_Y; ++j) {
    for (int i = 0; i < DIMENSION_X; ++i) {
      file << transitions[i][j][LEFT] << "\t";
    }
    file << endl;
  }

  while (!query.empty()) {
    pair <int, int> actual_point = *(query.begin());

    file << "Actual point: " << actual_point.first << " " << actual_point.second << endl;

    query.erase(query.begin());

    // check up
    if (get_transition(actual_point.first, actual_point.second, UP) && path[actual_point.first][actual_point.second
        - 1].empty()) {
      file << "UP is possible" << endl;
      path[actual_point.first][actual_point.second - 1] = path[actual_point.first][actual_point.second];
      path[actual_point.first][actual_point.second - 1].push_back(make_pair(actual_point.first, actual_point.second));
      query.push_back(make_pair(actual_point.first, actual_point.second - 1));
    }

    // check right
    if (get_transition(actual_point.first, actual_point.second, RIGHT)
        && path[actual_point.first + 1][actual_point.second].empty()) {
      file << "RIGHT is possible" << endl;
      path[actual_point.first + 1][actual_point.second] = path[actual_point.first][actual_point.second];
      path[actual_point.first + 1][actual_point.second].push_back(make_pair(actual_point.first, actual_point.second));
      query.push_back(make_pair(actual_point.first + 1, actual_point.second));
    }

    // check down
    if (get_transition(actual_point.first, actual_point.second, DOWN)
        && path[actual_point.first][actual_point.second + 1].empty()) {
      file << "DOWN is possible" << endl;
      path[actual_point.first][actual_point.second + 1] = path[actual_point.first][actual_point.second];
      path[actual_point.first][actual_point.second + 1].push_back(make_pair(actual_point.first, actual_point.second));
      query.push_back(make_pair(actual_point.first, actual_point.second + 1));
    }

    // check left
    if (get_transition(actual_point.first, actual_point.second, LEFT)
        && path[actual_point.first - 1][actual_point.second].empty()) {
      file << "LEFT is possible" << endl;
      path[actual_point.first - 1][actual_point.second] = path[actual_point.first][actual_point.second];
      path[actual_point.first - 1][actual_point.second].push_back(make_pair(actual_point.first, actual_point.second));
      query.push_back(make_pair(actual_point.first - 1, actual_point.second));
    }

    file << "Path: ";
    vector <pair <int, int> >::iterator it;
    for (it = path[actual_point.first][actual_point.second].begin(); it
        != path[actual_point.first][actual_point.second].end(); ++it)
      file << (*it).first << "-" << (*it).second << "; ";
    file << endl;

    if (actual_point.first == end.first && actual_point.second == end.second) {
      file.close();
      path[end.first][end.second].push_back(make_pair(end.first, end.second));
      path[end.first][end.second].erase(path[end.first][end.second].begin()); // Somehow the first step is duplicated... Fix it maybe?
      return path[end.first][end.second];
    }

  }
  file.close();
  return path[end.first][end.second];

}
;

Cell::Cell()
{
  x = y = width = height = 0;
}
;

Cell::Cell(int x_, int y_, int width_, int height_)
{
  x = x_;
  y = y_;
  width = width_;
  height = height_;
}
;

}//: namespace RecognizeLabyrinth
}//: namespace Processors
