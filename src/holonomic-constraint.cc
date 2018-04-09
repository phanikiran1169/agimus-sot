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

#include <../src/holonomic-constraint.hh>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>

namespace dynamicgraph {
  namespace sot {
    namespace hpp {
      using std::sin;
      using std::cos;
      double sinc (const double& t)
      {
        if (t < 1e-3) {
          const double t2 = t*t;
          return 1 - t2 / 6 + t2*t2 / 120;
        } else
          return sin (t) / t;
      }

      HolonomicConstraint::HolonomicConstraint (const std::string& name) :
        Entity (name),
        dim_ (0),
        g1SIN(NULL, "HolonomicConstraint("+name+")::input(double)::g1"),
        g2SIN(NULL, "HolonomicConstraint("+name+")::input(double)::g2"),
        g3SIN(NULL, "HolonomicConstraint("+name+")::input(double)::g3"),
        positionSIN   (NULL, "HolonomicConstraint("+name+")::input(matrixHomo)::position"),
        positionRefSIN(NULL, "HolonomicConstraint("+name+")::input(matrixHomo)::positionRef"),
        velocityRefSIN(NULL, "HolonomicConstraint("+name+")::input(vector)::velocityRef"),
        errorSOUT    ("HolonomicConstraint("+name+")::output(vector)::error"),
        controlSOUT  ("HolonomicConstraint("+name+")::output(vector)::control"),
        projectorSOUT("HolonomicConstraint("+name+")::output(matrix)::projector")
      {
        errorSOUT.setFunction
          (boost::bind (&HolonomicConstraint::computeError, this, _1, _2));
        controlSOUT.setFunction
          (boost::bind (&HolonomicConstraint::computeControl, this, _1, _2));
        projectorSOUT.setFunction
          (boost::bind (&HolonomicConstraint::computeProjector, this, _1, _2));
        signalRegistration (g1SIN);
        signalRegistration (g2SIN);
        signalRegistration (g3SIN);
        signalRegistration (positionSIN   );
        signalRegistration (positionRefSIN);
        signalRegistration (velocityRefSIN);
        signalRegistration (errorSOUT    );
        signalRegistration (controlSOUT  );
        signalRegistration (projectorSOUT);

        std::string docstring =
          "\n"
          "    Set the number of DoF of the robot\n";
        addCommand ("setNumberDoF", command::makeCommandVoid1
            (*this, &HolonomicConstraint::setNumberDoF, docstring));
      }

      Vector& HolonomicConstraint::computeError (Vector& error, const int& time)
      {
        const MatrixHomogeneous& oMp  = positionSIN   .access (time);
        const MatrixHomogeneous& oMpr = positionRefSIN.access (time);

        MatrixHomogeneous pMpr = oMp.inverse() * oMpr;
        Eigen::AngleAxisd aa (pMpr.linear());

        error.resize(6);
        error.head<3>() = pMpr.translation();
        error.tail<3>() = aa.angle() * aa.axis();

        return error;
      }

      Vector& HolonomicConstraint::computeControl   (Vector& control  , const int& time)
      {
        const Vector& vr = velocityRefSIN.access (time);
        const MatrixHomogeneous& oMp  = positionSIN   .access (time);
        const Vector& error = errorSOUT.  access (time);
        const double& g1 = g1SIN.access(time);
        const double& g2 = g2SIN.access(time);
        const double& g3 = g3SIN.access(time);

        control.resize (6);
        control.head<3>() = oMp.linear().row(0) * (vr[0] * cos (error[5]) + g1 * error[0]);
        control.tail<3>() = oMp.linear().row(2) * (vr[5] + g3 * error[5] + g2 * vr[0] * sinc(error[5]) * error[1]);

        assert (!control.hasNaN());
        return control;
      }

      Matrix& HolonomicConstraint::computeProjector (Matrix& projector, const int& )
      {
        projector.resize(6, dim_);
        projector.leftCols<6>().setIdentity();
        projector.rightCols(dim_-6).setZero();
        return projector;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HolonomicConstraint, "HolonomicConstraint");
    } // namespace hpp
  } // namespace sot
} // namespace dynamicgraph
