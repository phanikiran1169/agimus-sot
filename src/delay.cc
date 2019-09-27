// Copyright 2018 CNRS - Airbus SAS
// Author: Joseph Mirabel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
