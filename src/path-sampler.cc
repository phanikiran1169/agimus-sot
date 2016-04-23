//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of sot-hpp
// sot-hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot-hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-hpp  If not, see
// <http://www.gnu.org/licenses/>.

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-setter.h>
#include <hpp/model/device.hh>
#include <hpp/model/urdf/util.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/path-vector.hh>
#include <sot/hpp/config.hh>
#include "path-sampler.hh"
#include "command.hh"
#include <boost/assign/list_inserter.hpp> // for 'push_back()'
#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      using dynamicgraph::Entity;
      using ::hpp::model::Device;
      using ::hpp::model::DevicePtr_t;
      using ::hpp::core::PathVector;
      using ::hpp::core::SteeringMethodStraight;
      using ::hpp::core::SteeringMethodPtr_t;
      using ::hpp::model::ConfigurationIn_t;
      using ::hpp::model::ConfigurationOut_t;
      using ::hpp::model::Configuration_t;
      using ::hpp::model::size_type;
      // Convert configuration from hpp::model::Configuration_t to
      // dynamicgraph::Vector
      void convert (ConfigurationIn_t hppConfig, Vector& sotConfig)
      {
	for (size_type i=0; i<hppConfig.size (); ++i) {
	  sotConfig (i) = hppConfig [i];
	}
      }

      // Convert configuration from hpp::model::Configuration_t to
      // dynamicgraph::Vector
      void convert (const Vector& sotConfig, ConfigurationOut_t hppConfig)
      {
	for (size_type i=0; i<hppConfig.size (); ++i) {
	  hppConfig [i] = sotConfig (i);
	}
      }

      void convert ( Vector& sotConfig)
      {
	Vector temp;
	temp.resize(sotConfig.size ());
	sotConfig.extract(0,36,temp);
	sotConfig.resize(sotConfig.size()+3);
	sotConfig(0) = temp(0);
	sotConfig(1) = temp(1);
	sotConfig(5) = temp(2);
	sotConfig(2) = 0;
	sotConfig(3) = 0;
	sotConfig(4) = 0;
	for (size_type i=6; i<sotConfig.size (); ++i) {
	  sotConfig ((unsigned int)i) = temp ((unsigned int)(i-3));
	}
      }

      PathSampler::PathSampler (const std::string& name) :
	Entity (name), configurationSOUT
	("PathSampler("+name+")::output(vector)::configuration"),
	jointPositionSIN(NULL,"PathSampler("+name+")::input(vector)::position"),
	robot_ (), problem_ (), path_ (), steeringMethod_ (), timeStep_ (),
	lastWaypoint_ (0), state_ (NOT_STARTED), rootJointType_("planar"),
	startTime_ (0)
      {
	using command::makeCommandVoid0;
	using command::makeDirectSetter;
	// Initialize input signal
	signalRegistration(jointPositionSIN);
	signalRegistration (configurationSOUT);

	configurationSOUT.setFunction
	  (boost::bind (&PathSampler::computeConfiguration, this, _1, _2));

	std::string docstring;
	// Add command AddWaypoint
	docstring =
	  "\n"
	  "    Add a way point to the path\n"
	  "      input (vector) the waypoint: should be of the same dimension as\n"
	  "                     robot configuration\n"
	  "\n"
	  "      Path is a concatenation of straight interpolation paths as "
	  "defined\n"
	  "      in hpp::core::PathVector.\n";
	addCommand (std::string ("addWaypoint"),
		    new AddWaypoint (*this, docstring));

	docstring =
	  "\n"
	  "    Start sampling path\n";
	addCommand ("start", makeCommandVoid0(*this, &PathSampler::start,
					     docstring));

	docstring =
	  "\n"
	  "    Reset path\n";
	addCommand ("resetPath", makeCommandVoid0 (*this,
						   &PathSampler::resetPath,
						   docstring));

	docstring =
	  "\n"
	  "    Set timeStep\n"
	  "      Input: float\n";
	addCommand ("setTimeStep",
		    makeDirectSetter (*this, &timeStep_, docstring));

	docstring =
	  "\n"
	  "    Load urdf robot model\n"
	  "      Input: string packageName: name of the ros package containing\n"
	  "               the urdf file\n"
	  "             string rootJointType: type of the root joint among "
	  "['freeflyer',\n"
	  "               'planar', 'anchor']\n"
	  "             string modelName: name of the urdf file\n"
	  "\n"
	  "     The url of the file is 'package://${packageName}/urdf/"
	  "modelName.urdf'\n";
	addCommand ("loadRobotModel",
		    dynamicgraph::command::makeCommandVoid3
		    (*this, &PathSampler::loadRobotModel, docstring));
      }

      void PathSampler::loadRobotModel (const std::string& packageName,
					const std::string& rootJointType,
					const std::string& modelName)
      {
	robot_ = Device::create ("modelName");
	::hpp::model::urdf::loadRobotModel (robot_, rootJointType, packageName,
					    modelName, "", "");
	// Create a new empty path
	path_ = PathVector::create (robot_->configSize ());
	steeringMethod_ = SteeringMethodPtr_t (new SteeringMethodStraight
					       (robot_));
      }

      void PathSampler::addWaypoint (const Vector& wp)
      {
	if (!robot_) {
	  throw std::runtime_error ("Robot is not initialized");
	}
	if (robot_->configSize () != (size_type) wp.size ()) {
	  std::ostringstream oss ("                     \n");
	  oss << "Dimension of robot (" << robot_->configSize () <<
	    ") and dimension of waypoint (" << wp.size () << ") differ.";
	  throw std::runtime_error (oss.str ().c_str ());
	}
	Configuration_t waypoint (wp.size ());
	convert (wp, waypoint);
	if (lastWaypoint_.size () != 0) {
	  // It is not the first way point, add a straight path
	  path_->appendPath ((*steeringMethod_) (lastWaypoint_, waypoint));
	}
	lastWaypoint_ = waypoint;
	if (state_ == RESET) {
          state_ = NOT_STARTED;
        }

      }

      void PathSampler::start ()
      {
	// if already started or finished, do nothing
	if (state_ == NOT_STARTED) {
	  startTime_ = configurationSOUT.getTime ();
	  state_ = SAMPLING;
	}
      }

      void PathSampler::resetPath ()
      {
	if (robot_) {
	  path_ = PathVector::create (robot_->configSize ());
	}
	// Keep last waypoint
	state_ = NOT_STARTED;
      }


      Vector& PathSampler::computeConfiguration
      (Vector& configuration, const int& time)
      {
	if (!path_) {
	  throw std::runtime_error
	    ("Path is not initialized in entity PathSampler (" +
	     getName () + ")");
	}
	if (path_->length () == 0) {
	  if (lastWaypoint_.size () != robot_->configSize ()) {
	    throw std::runtime_error
	      ("Path length is 0 in entity PathSampler (" +
	       getName () + ") and waypoint size does not fit robot size");
	  }
	  configuration = jointPositionSIN(time);
	  return configuration;
	}
	configuration.resize (path_->outputSize ());
	if (state_ == NOT_STARTED) {

	  //convert ((*path_) (0), configuration);
	  configuration = jointPositionSIN(time);
	}
	else if (state_ == FINISHED) {
	  convert ((*path_) (path_->length ()), configuration);
	}
	else if (state_ == SAMPLING) {
	  double t = timeStep_ * (time - startTime_);
	  if (t > path_->length ()) {
	    t = path_->length ();
	    state_ = FINISHED;
	  }
	  convert ((*path_) (t), configuration);
	}
	if(rootJointType_ == "planar") {
	  convert (configuration);
	}
	return configuration;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (PathSampler, "PathSampler");
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
