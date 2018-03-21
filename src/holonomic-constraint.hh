// Copyright (c) 2018, Joseph Mirabel
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

#ifndef SOT_HPP_HOLONOMIC_CONSTRAINT_HH
# define SOT_HPP_HOLONOMIC_CONSTRAINT_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/hpp/config.hh>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      /// Holonomic constraints
      class SOT_HPP_DLLAPI HolonomicConstraint : public dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

        HolonomicConstraint (const std::string& name);

        ~HolonomicConstraint () {}

        /// Header documentation of the python class
        virtual std::string getDocString () const
        {
          return
            "Compute the control (v, w) of size 2 for a mobile base.\n"
            "  Signal velocityRef is the desired velocity (v_r, w_r) expressed in the desired frame.\n";
        }

        void setNumberDoF (const unsigned int& d)
        { dim_ = d; }

        private:
        Vector& computeError     (Vector& error    , const int& time);
        Vector& computeControl   (Vector& control  , const int& time);
        Matrix& computeProjector (Matrix& projector, const int& time);

        unsigned int dim_;

        SignalPtr <double, int> g1SIN;
        SignalPtr <double, int> g2SIN;
        SignalPtr <double, int> g3SIN;

        SignalPtr <Vector, int> positionSIN;
        SignalPtr <Vector, int> positionRefSIN;
        SignalPtr <Vector, int> velocityRefSIN;

        Signal <Vector, int> errorSOUT;
        Signal <Vector, int> controlSOUT;
        Signal <Matrix, int> projectorSOUT;
      };
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
#endif // SOT_HPP_HOLONOMIC_CONSTRAINT_HH
