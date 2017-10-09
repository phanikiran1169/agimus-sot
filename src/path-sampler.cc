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
#include <sot/hpp/config.hh>
#include "path-sampler.hh"
#include <boost/assign/list_inserter.hpp> // for 'push_back()'
#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      using dynamicgraph::Entity;
      using ::hpp::floatSeq;
      using ::hpp::floatSeq_var;
      using ::hpp::floatSeq_out;

      // Convert configuration from floatSeq to
      // dynamicgraph::Vector
      void convert (const floatSeq& hppConfig, Vector& sotConfig)
      {
	for (CORBA::ULong i=0; i<hppConfig.length (); ++i) {
	  sotConfig ((unsigned int)i) = hppConfig [i];
	}
      }

      // Convert configuration from floatSeq to
      // dynamicgraph::Vector
      void convert (const Vector& sotConfig, floatSeq_out hppConfig)
      {
	for (int i=0; i<sotConfig.size (); ++i) {
	  hppConfig [i] = sotConfig ((unsigned int)i);
	}
      }

      void convert_planar ( Vector& sotConfig)
      {
	Vector temp;
	temp.resize(sotConfig.size ());
        temp = sotConfig;
	sotConfig.resize(sotConfig.size()+2);
        sotConfig.tail(temp.size()) = temp;

        // position
	sotConfig.head<2>() = temp.head<2>();
	sotConfig(2) = 0;

        // Roll = Pitch = 0
        // Yaw = atan( sin(theta) / cos(theta) )
	sotConfig(3) = 0;
	sotConfig(4) = 0;
	sotConfig(5) = std::atan2 (temp(3), temp(2));
      }

      void convert_freeflyer ( Vector& sotConfig)
      {
	Vector temp;
	temp.resize(sotConfig.size ());
        temp = sotConfig;
	sotConfig.resize(sotConfig.size()-1);
        sotConfig.tail(temp.size() - 7) = temp.tail(temp.size() - 7);

        // position
	sotConfig.head<3>() = temp.head<3>();

        // TODO
        // Compute RPY from quaternion in temp[3:7]
        // for now Roll = Pitch = Yaw = 0
        sotConfig.segment<3>(3).setZero();
      }

      PathSampler::PathSampler (const std::string& name) :
	Entity (name), configurationSOUT
	("PathSampler("+name+")::output(vector)::configuration"),
	jointPositionSIN(NULL,"PathSampler("+name+")::input(vector)::position"),
        hpp_ (0, NULL), timeStep_ (),
        pathLength_(0), pathId_ (-1), sizeOutput_(0),
        state_ (NOT_STARTED), rootJointType_("anchor"),
	startTime_ ()
      {
	using command::makeCommandVoid0;
	using command::makeCommandVoid1;
	using command::makeCommandVoid2;
	using command::makeDirectSetter;
	// Initialize input signal
	signalRegistration(jointPositionSIN);
	signalRegistration (configurationSOUT);

	configurationSOUT.setFunction
	  (boost::bind (&PathSampler::computeConfiguration, this, _1, _2));

	std::string docstring;

	docstring =
	  "\n"
	  "    Start sampling path\n";
	addCommand ("start", makeCommandVoid0
            (*this, &PathSampler::start, docstring));

	docstring =
	  "\n"
	  "    Reset path\n";
	addCommand ("resetPath", makeCommandVoid0
            (*this, &PathSampler::resetPath, docstring));

	docstring =
	  "\n"
	  "    Set path ID in HPP server\n";
	addCommand ("setPathID", makeCommandVoid1
            (*this, &PathSampler::setPathID, docstring));

	docstring =
	  "\n"
	  "    Set timeStep\n"
	  "      Input: float\n";
	addCommand ("setTimeStep", makeDirectSetter
            (*this, &timeStep_, docstring));

	docstring =
	  "\n"
	  "    Set the root joint type\n"
	  "      Input: string root_joint_type: one of ['freeflyer', 'planar', 'anchor']\n";
	addCommand ("setRootJointType", makeDirectSetter
            (*this, &rootJointType_, docstring));

	docstring =
	  "\n"
	  "    Connect to HPP corba server\n"
	  "      Input: string host\n"
	  "             int    port\n";
        addCommand ("connect", makeCommandVoid2
            (*this, &PathSampler::connect, docstring));

	docstring =
	  "\n"
	  "    Add parameter selection in the HPP configuration\n";
	addCommand ("addParamSelection", makeCommandVoid2
            (*this, &PathSampler::addParamSelection, docstring));
      }

      PathSampler::~PathSampler ()
      {
      }

      void PathSampler::connect (const std::string& host, const int& port)
      {
        std::stringstream ss;
        ss << "corbaloc:iiop:" << host << ':' << port << "/NameService";
        hpp_.connect (ss.str().c_str());
      }

      void PathSampler::setPathID (const int& id)
      {
        const int limit = hpp_.problem()->numberPaths();
	if (id >= limit) {
          std::stringstream ss;
          ss << "Path ID exceeds limit (" << limit << ").";
	  throw std::runtime_error (ss.str());
	}
        pathId_ = id;
        pathLength_ = hpp_.problem()->pathLength((CORBA::UShort)pathId_);
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
        pathId_ = -1;
	state_ = RESET;
      }

      Vector& PathSampler::computeConfiguration
      (Vector& configuration, const int& time)
      {
        if (pathId_ < 0) {
          throw std::runtime_error
            ("Path is not initialized in entity PathSampler (" +
             getName () + ")");
        }
        if (pathLength_ == 0) {
          configuration = jointPositionSIN(time);
          return configuration;
        }
        Vector q (hpp_.robot()->getConfigSize());
        if (state_ == NOT_STARTED) {
          configuration = jointPositionSIN(time);
          return configuration;
        }
        else if (state_ == FINISHED) {
          floatSeq_var config = hpp_.problem()->configAtParam((CORBA::UShort)pathId_, pathLength_);
          convert (config.in(), q);
        }
        else if (state_ == SAMPLING) {
          double t = timeStep_ * (time - startTime_);
          if (t > pathLength_) {
            t = pathLength_;
            state_ = FINISHED;
          }
          floatSeq_var config = hpp_.problem()->configAtParam((CORBA::UShort)pathId_, t);
          convert (config.in(), q);
        }
        // configuration.resize ();
        applySelection(q, configuration);
        if(rootJointType_ == "planar") {
          convert_planar (configuration);
        } else if(rootJointType_ == "freeflyer") {
          convert_freeflyer (configuration);
        }
        return configuration;
      }

      void PathSampler::addParamSelection (const int& start, const int& length)
      {
        selection_.push_back (segment_t(start, length));
        sizeOutput_ += length;
      }

      void PathSampler::applySelection (const Vector& in, Vector& out)
      {
        if (selection_.empty()) out = in;
        out.resize(sizeOutput_);
        int r = 0;
        for (std::size_t i = 0; i < selection_.size(); ++i) {
          out.segment(r, selection_[i].second)
            = in.segment(selection_[i].first, selection_[i].second);
          r += selection_[i].second;
        }
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (PathSampler, "PathSampler");
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
