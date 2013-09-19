/*!
 * \file
 * \brief
 * \author Emil Natil
 */

#include <memory>
#include <string>

#include "LabyrinthReading.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace LabyrinthReading {

LabyrinthReading::LabyrinthReading(const std::string & name) :
		Base::Component(name)  {

}

LabyrinthReading::~LabyrinthReading() {
}

void LabyrinthReading::prepareInterface() {
  LOG(LNOTICE) << "LabyrinthReading::prepareInterface\n";
	// Register data streams, events and event handlers HERE!

  h_onNewData.setup(this, &LabyrinthReading::onNewData);
  registerHandler("onNewData", &h_onNewData);
  addDependency("onNewData", &stringReading);

  registerStream("stringReading", &stringReading);
  registerStream("out_reading", &out_reading);
}

bool LabyrinthReading::onInit() {
  LOG(LNOTICE) << "LabyrinthReading::onInit\n";

	return true;
}

bool LabyrinthReading::onFinish() {
  LOG(LNOTICE) << "LabyrinthReading::onFinish\n";
	return true;
}

bool LabyrinthReading::onStop() {
  LOG(LNOTICE) << "LabyrinthReading::onStop\n";
	return true;
}

bool LabyrinthReading::onStart() {
  LOG(LNOTICE) << "LabyrinthReading::onStart\n";

	return true;
}

void LabyrinthReading::onNewData() {
  //LOG(LNOTICE) << "LabyrinthReading::onNewData\n";
  string stringDataReceived = stringReading.read();

  Types::Mrrocpp_Proxy::LabyrinthData data;

  istringstream iss(stringDataReceived);


  iss >> data.labyrinth_solved;
  iss >> data.calibrate_labyrinth;
  iss >> data.path_size;
  iss >> data.start_point_x;
  iss >> data.start_point_y;
  iss >> data.end_point_x;
  iss >> data.end_point_y;
  int i = 0;
  do
  {
    iss >> data.path[i];
    ++i;
  } while (iss);

  out_reading.write(data);
}



} //: namespace LabyrinthReading
} //: namespace Processors
