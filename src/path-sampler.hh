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

#ifndef SOT_HPP_PATH_SAMPLER_HH
# define SOT_HPP_PATH_SAMPLER_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <hpp/corbaserver/client.hh>
#include <sot/hpp/config.hh>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      /// Sample a path at a given time step
      class SOT_HPP_DLLAPI PathSampler : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();
        // State of the sampler
        enum State {
          NOT_STARTED = 0,
          RESET,
          SAMPLING,
          FINISHED
        };
        PathSampler (const std::string& name);
        ~PathSampler ();

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Sample a path at a given time step\n\n"
            "  Path is a list of straight interpolations in the robot "
            "configuration space.\n"
            "The robot kinematic chain is built from a urdf file.\n";
        }
        // Start sampling path
        void start ();
        // Reset path
        void resetPath ();
        // Set path
        void setPathID (const int& id);
        // Connect to the HPP server
        void connect (const std::string& host, const int& port = 2809);
        private:
        Vector& computeConfiguration (Vector& configuration, const int& time);
        Signal <Vector, int> configurationSOUT;
        SignalPtr<Vector,int> jointPositionSIN;
        ::hpp::corbaServer::Client hpp_;

        double timeStep_, pathLength_;
        int pathId_;
        State state_;
        std::string rootJointType_;
        int startTime_;
      };
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
#endif // SOT_HPP_PATH_SAMPLER_HH
