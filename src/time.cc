// Copyright (c) 2018, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of agimus-sot.
// agimus-sot is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// agimus-sot is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// agimus-sot. If not, see <http://www.gnu.org/licenses/>.

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
  namespace agimus {
      /// Time
      template <typename TimeT = int>
      class AGIMUS_SOT_DLLAPI Time : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

        public:
        Time (const std::string& name) :
          Entity (name),
          time (0),
          cur ("Time("+name+")::output(int)::now"),
          sup ("Time("+name+")::output(bool)::after"),
          inf ("Time("+name+")::output(bool)::before")
        {
          cur.setFunction (boost::bind (&Time::now , this, _1, _2));
          sup.setFunction (boost::bind (&Time::aft , this, _1, _2));
          inf.setFunction (boost::bind (&Time::bef , this, _1, _2));
          signalRegistration (cur << sup << inf);

          using command::makeCommandVoid1;
          std::string docstring =
            "\n"
            "    Set time to be compared with\n";
          addCommand ("setTime", makeCommandVoid1
              (*this, &Time::setTime, docstring));
        }

        ~Time () {}

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Time tools in SoT.\n"
            "The meaning of the signals is:\n"
            "- now: provides the current time.\n"
            "- after: true when time > internalTime.\n"
            "- before: true when time < internalTime.\n"
            ;
        }

        void setTime (const TimeT& t) { time = t; }

        private:
        TimeT& now (TimeT& res, const TimeT& t) { res =  t        ; return res; }
        bool & aft (bool & res, const TimeT& t) { res = (t > time); return res; }
        bool & bef (bool & res, const TimeT& t) { res = (t < time); return res; }

        TimeT time;
        Signal <TimeT, TimeT> cur;
        Signal <bool, TimeT> sup, inf;
      };

      typedef Time<int> _Time;
      template<>
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (_Time, "Time");
  } // namespace agimus
} // namespace dynamicgraph
