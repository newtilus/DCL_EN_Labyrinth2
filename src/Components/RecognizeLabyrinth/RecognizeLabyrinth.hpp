/*!
 * \file
 * \brief 
 * \author Emil Natil
 */

#ifndef RECOGNIZE_LABYRINTH_HPP_
#define RECOGNIZE_LABYRINTH_HPP_

#define DIMENSION_X 8
#define DIMENSION_Y 9
#define MAX_PATH 48
#define DOWN 0
#define RIGHT 1
#define UP 2
#define LEFT 3
#define HORIZONTAL 0
#define VERTICAL 1
#define START_X 0
#define START_Y 0
#define END_X 7
#define END_Y 8


#include "EventHandler2.hpp"

#include <cv.h>
#include <highgui.h>
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "Panel_Empty.hpp"
#include "DataStream.hpp"
#include "Property.hpp"


namespace Processors {
namespace RecognizeLabyrinth {

using namespace cv;

/*!
 * \class RecognizeLabyrinth
 */
class RecognizeLabyrinth: public Base::Component
{
public:

  RecognizeLabyrinth();
  RecognizeLabyrinth(const std::string & name = "");
  virtual ~RecognizeLabyrinth();

protected:

  /*!
   * Connects source to given device.
   */
  bool onInit();

  /*!
   * Disconnect source from device, closes streams, etc.
   */
  bool onFinish();

  /*!
   * Retrieves data from device.
   */
  bool onStep();

  /*!
   * Start component
   */
  bool onStart();

  /*!
   * Stop component
   */
  bool onStop();

  /*!
   * Event handler function.
   */
  void onProcess();
  void processLabyrinth();

  /*!
   * Event handler function.
   */
  void onNewImage();

  /*!
   * Event handler function.
   */
  void onRpcCall();

  void prepareInterface();

  /// handlers
  Base::EventHandler <RecognizeLabyrinth> h_onNewImage; // new image of labyrinth arrived
  Base::EventHandler <RecognizeLabyrinth> h_onProcess; // process signal from initiation component

  /// inputs
  //Base::DataStreamIn <Mat, Base::DataStreamBuffer::Newest> in_img; // new image of labyrinth
  Base::DataStreamIn <Mat, Base::DataStreamBuffer::Newest> in_img;
  Base::DataStreamIn <bool, Base::DataStreamBuffer::Newest> in_start_processing;

  /// outputs
  Base::DataStreamOut <Mat> out_img;  //new image of solved labyrinth
  Base::DataStreamOut <string> out_reading;  //data
  Base::DataStreamOut <bool> out_done_processing;

  // Load calibration parameters from file.
  bool loadParameters();

private:
  // Properties
  Base::Property<int> prop_width;
  Base::Property<int> prop_height;
  //Base::Property<string> prop_calibrationResults;
  Base::Property<int> prop_x1;
  Base::Property<int> prop_x2;
  Base::Property<int> prop_y1;
  Base::Property<int> prop_y2;
  Base::Property<int> prop_threshold;
  Base::Property<int> prop_segmentation_threshold;
  Base::Property<int> prop_min_length;
  Base::Property<int> prop_max_gap;
  Base::Property<int> prop_calibrate_labyrinth;

  cv::Mat hue_img;
  cv::Mat saturation_img;
  cv::Mat value_img;
  cv::Mat segments_img;
  // The 3x3 camera matrix containing focal lengths fx,fy and displacement of the center of coordinates cx,cy.
  cv::Mat cameraMatrix;
  // Vector with distortion coefficients k_1, k_2, p_1, p_2, k_3.
  cv::Mat distCoeffs;
  // Matrices storing partial undistortion results.
  //Mat map1, map2;
  Mat labyrinth_first;
  Mat labyrinth_last;
  Mat first_image;
  int k;
  bool showImage;
  bool showProcessed;
  bool solve;
  bool calibrate;
  int x1; // start point in labyrinth x
  int x2; // end point in labyrinth x
  int y1; // start point in labyrinth y
  int y2; // end point in labyrinth y
  int min_x;
  int max_x;
  int min_y;
  int max_y;
  double rotation;
  int segmentation_threshold;
  int threshold;
  int min_length;
  int calibrate_labyrinth;
  int max_gap;
  // Indicates if the file with calibration parameters was found
  bool file_found;
  bool first_image_saved;
  bool labyrinth_found;
  bool labyrinth_solved;
  int path[MAX_PATH];
  int path_size;
//  int start_point[];
//  int end_point[];
};

class Cell
{

private:

public:
  int x;
  int y;
  int height;
  int width;

  Cell();
  Cell(int x, int y, int width, int height);

};

class Labyrinth
{
private:
  bool transitions[DIMENSION_X][DIMENSION_Y][4]; // array representing possible moves
  vector <pair <int, int> > path[DIMENSION_X][DIMENSION_Y]; // array representing path leading to the point
  vector <pair <int, int> > query; // vector of point waiting to be checked
  bool waiting[DIMENSION_X][DIMENSION_Y]; // array representing the fact that cell is waiting to be checked
  Cell* cells[DIMENSION_X][DIMENSION_Y];

public:
  Labyrinth();
  //~Labyrinth();
  bool insert_cell(int pos_x, int pos_y, int x, int y, int width, int heigth);
  Cell* get_cell(int pos_x, int pos_y);
  void set_transition(int pos_x, int pos_y, int direction);
  void remove_transition(int pos_x, int pos_y, int direction);
  bool get_transition(int pos_x, int pos_y, int direction);
  bool get_waiting(int pos_x, int pos_y);
  bool set_waiting(int pos_x, int pos_y);
  vector <pair <int, int> > solve(pair <int, int> start, pair <int, int> end);

};

}//: namespace RecognizeLabyrinth
}//: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RecognizeLabyrinth", Processors::RecognizeLabyrinth::RecognizeLabyrinth)

#endif /* RecognizeLabyrinth_HPP_ */
