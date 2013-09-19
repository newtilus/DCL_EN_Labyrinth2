/*
 *
 *
 *
 *
 */

#ifndef LABYRINTH_DATA_HPP_
#define LABYRINTH_DATA_HPP_

#define MAX_PATH_SIZE 64

#include "Reading.hpp"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */


class LabyrinthData: public Reading
{
public:
	LabyrinthData()
	{
	}

	virtual ~LabyrinthData()
	{
	}

	virtual LabyrinthData* clone()
	{
		return new LabyrinthData(*this);
	}

  bool labyrinth_solved;
  int path_size;
  int start_point_x;
  int start_point_y;
  int end_point_x;
  int end_point_y;
  int path[MAX_PATH_SIZE];
  bool calibrate_labyrinth;

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		LOG(LWARNING) << "PBReading::serialize()\n";
		ar & boost::serialization::base_object <Reading>(*this);

    ar & labyrinth_solved;
    ar & path_size;
    ar & start_point_x;
    ar & start_point_y;
    ar & end_point_x;
    ar & end_point_y;
    ar & path;
    ar & calibrate_labyrinth;
	}
};


}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* LABYRINTH_DATA_HPP_ */
