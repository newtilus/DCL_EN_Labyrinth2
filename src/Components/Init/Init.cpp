/*!
 * \file
 * \brief
 * \author Emil Natil
 */

#include <memory>
#include <string>

#include "Init.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace std;

namespace Processors {
namespace Init {

Init::Init(const std::string & name) :
		Base::Component(name)  {

}

Init::~Init() {
}

void Init::prepareInterface() {
	// Register data streams, events and event handlers HERE!
  h_onDoneProcessing.setup(this, &Init::onDoneProcessing);

  registerHandler("onDoneProcessing", &h_onDoneProcessing);
  addDependency("onDoneProcessing", &in_done_processing);

  registerStream("in_done_processing", &in_done_processing);
  registerStream("out_start_processing", &out_start_processing);

}

bool Init::onInit() {
  cout << "Component Init::onInit()" << endl;
  out_start_processing.write(true);
  return true;
}

bool Init::onFinish() {
	return true;
}

bool Init::onStop() {
	return true;
}

bool Init::onStart() {
  cout << "Component Init::onStart()" << endl;

  char type;
  do
  {
      cout << "Should I start? [y]" << endl;
      cin >> type;
      if(type == 'y') {
        out_start_processing.write(true);
        return true;
      }

  }
  while( !cin.fail());

  return true;
}

void Init::onDoneProcessing() {
  bool done = in_done_processing.read();

  char type;
  if(!done)
  {
    out_start_processing.write(true);
    return;
  }

  do
  {
      cout << "Repeat processing? [y]" << endl;
      cin >> type;
      if(type == 'y') {
        out_start_processing.write(true);
        return;
      }

  }
  while( !cin.fail());

}



} //: namespace Init
} //: namespace Processors
