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

#ifndef SOT_HPP_COMMAND_HH
# define SOT_HPP_COMMAND_HH

# include <boost/assign.hpp>
# include <dynamic-graph/command.h>
# include "path-sampler.hh"

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;

      class AddWaypoint : public Command
      {
      public:
	AddWaypoint (PathSampler& entity, const std::string& docString) :
	  Command (entity, boost::assign::list_of (Value::VECTOR), docString)
	{
	}
	virtual Value doExecute ()
	{
	  PathSampler& ps = static_cast <PathSampler&> (owner ());
	  std::vector<Value> values = getParameterValues();
	  Vector waypoint = values[0].value();
	  ps.addWaypoint (waypoint);
	  return Value();
	}
      }; // class AddWaypoint
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
#endif // SOT_HPP_COMMAND_HH
