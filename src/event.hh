// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of sot_hpp.
// sot_hpp is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot_hpp is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot_hpp. If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_HPP_EVENT_HH
# define SOT_HPP_EVENT_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/command-bind.h>

#include <sot/hpp/config.hh>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      /// Event
      class SOT_HPP_DLLAPI Event : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

        Event (const std::string& name) :
          Entity (name),
          checkSOUT ("Event("+name+")::output(bool)::check"),
          conditionSIN(NULL,"Event("+name+")::input(bool)::condition")
        {
          checkSOUT.setFunction
            (boost::bind (&Event::check, this, _1, _2));
          signalRegistration (conditionSIN);
          signalRegistration (checkSOUT);

          using command::makeCommandVoid1;
          std::string docstring =
            "\n"
            "    Add a signal\n";
          addCommand ("addSignal", makeCommandVoid1
              (*this, &Event::addSignal, docstring));
        }

        ~Event () {}

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Send an event when the input changes\n\n"
            "  The signal triggered is called whenever the condition is satisfied.\n";
        }

        void addSignal (const std::string& signal)
        {
          std::istringstream iss (signal);
          triggers.push_back(&PoolStorage::getInstance()->getSignal (iss));
        }

        private:
        typedef SignalBase<int>* Trigger_t;
        typedef std::vector<Trigger_t> Triggers_t;

        bool& check (bool& ret, const int& time)
        {
          const bool& val = conditionSIN (time);
          ret = (val != lastVal_);
          if (ret) {
            lastVal_ = val;
            for (Triggers_t::const_iterator _s = triggers.begin();
                _s != triggers.end(); ++_s)
              (*_s)->recompute (time);
            std::cout << "trigger: " << val << std::endl;
          }
          return ret;
        }

        Signal <bool, int> checkSOUT;

        Triggers_t triggers;
        SignalPtr <bool, int> conditionSIN;

        bool lastVal_;
      };
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
#endif // SOT_HPP_PATH_SAMPLER_HH
