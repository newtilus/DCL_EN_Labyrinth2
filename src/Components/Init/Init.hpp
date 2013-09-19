/*!
 * \file
 * \brief 
 * \author Emil Natil
 */

#ifndef INIT_HPP_
#define INIT_HPP_

#include <cv.h>

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace Init {

using namespace cv;

/*!
 * \class Init
 * \brief Init processor class.
 *
 * Init processor.
 */
class Init: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Init(const std::string & name = "Init");

	/*!
	 * Destructor
	 */
	virtual ~Init();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

  /// handlers
  Base::EventHandler <Init> h_onDoneProcessing; // new image of labyrinth arrived

  /// inputs
  //Base::DataStreamIn <Mat, Base::DataStreamBuffer::Newest> in_img; // new image of labyrinth
  Base::DataStreamIn <bool, Base::DataStreamBuffer::Newest> in_done_processing;

  /// outputs
  Base::DataStreamOut <bool> out_start_processing;  //new image of solved labyrinth

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
  void onDoneProcessing();

};

} //: namespace Init
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Init", Processors::Init::Init)

#endif /* INIT_HPP_ */
