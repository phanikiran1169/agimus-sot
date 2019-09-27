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
