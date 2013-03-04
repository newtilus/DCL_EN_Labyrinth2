/*!
 * \file
 * \brief 
 * \author Emil Natil
 */

#ifndef LABYRINTHREADING_HPP_
#define LABYRINTHREADING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/Mrrocpp_Proxy/LabyrinthData.hpp"



namespace Processors {
namespace LabyrinthReading {

/*!
 * \class LabyrinthReading
 * \brief LabyrinthReading processor class.
 *
 * LabyrinthReading processor.
 */
class LabyrinthReading: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	LabyrinthReading(const std::string & name = "LabyrinthReading");

	/*!
	 * Destructor
	 */
	virtual ~LabyrinthReading();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

  /// handlers
  Base::EventHandler <LabyrinthReading> h_onNewData; // new data from labyrinth solver arrived

  // streams
  Base::DataStreamIn<string> stringReading;
  Base::DataStreamOut<Types::Mrrocpp_Proxy::LabyrinthData> out_reading;



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

	/*
	 * Handle new incoming data from labyrinth solver
	 */
	void onNewData();

};

} //: namespace LabyrinthReading
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("LabyrinthReading", Processors::LabyrinthReading::LabyrinthReading)

#endif /* LABYRINTHREADING_HPP_ */
