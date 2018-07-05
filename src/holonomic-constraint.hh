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

#ifndef AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH
# define AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
  namespace agimus {
      /// Holonomic constraints
      class AGIMUS_SOT_DLLAPI HolonomicConstraint : public dynamicgraph::Entity
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

        SignalPtr <sot::MatrixHomogeneous, int> positionSIN;
        SignalPtr <sot::MatrixHomogeneous, int> positionRefSIN;
        SignalPtr <Vector, int> velocityRefSIN;

        Signal <Vector, int> errorSOUT;
        Signal <Vector, int> controlSOUT;
        Signal <Matrix, int> projectorSOUT;
      };
  } // namespace agimus
} // namespace dynamicgraph
#endif // AGIMUS_SOT_HOLONOMIC_CONSTRAINT_HH
