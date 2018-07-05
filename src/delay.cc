// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of agimus_sot.
// agimus_sot is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// agimus_sot is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// agimus_sot. If not, see <http://www.gnu.org/licenses/>.

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
  namespace agimus {
      /// Delay
      template <typename Value, typename Time = int>
      class AGIMUS_SOT_DLLAPI Delay : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

        Delay (const std::string& name) :
          Entity (name),
          sin(NULL,"Delay("+name+")::input(unspecified)::sin"),
          currentOUT ("Delay("+name+")::output(unspecified)::current"),
          previousOUT("Delay("+name+")::output(unspecified)::previous")
        {
          currentOUT .setFunction (boost::bind (&Delay::current , this, _1, _2));
          previousOUT.setFunction (boost::bind (&Delay::previous, this, _1, _2));
          signalRegistration (sin << currentOUT << previousOUT);

          using command::makeCommandVoid1;
          std::string docstring =
            "\n"
            "    Set current value in memory\n";
          addCommand ("setMemory", makeCommandVoid1
              (*this, &Delay::setMemory, docstring));
        }

        ~Delay () {}

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Enable delayed signal in SoT.\n"
            "Signal previous at time t+1 will return the value of signal current "
            " at time <= t (the last time signal current was called).";
        }

        void setMemory (const Value& val)
        {
          mem = val;
        }

        private:
        Value& current (Value& res, const Time& time)
        {
          previousOUT.access (time);
          res = sin.access (time);
          mem = res;
          return res;
        }

        Value& previous (Value& res, const Time&)
        {
          res = mem;
          return res;
        }

        Value mem;
        SignalPtr <Value, Time> sin;
        Signal <Value, Time> currentOUT, previousOUT;
      };

      typedef Delay<Vector,int> DelayVector;
      template<>
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (DelayVector, "DelayVector");
  } // namespace agimus
} // namespace dynamicgraph
