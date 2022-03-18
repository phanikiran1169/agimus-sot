// Copyright 2022 CNRS - Toward SAS
// Author: Florent Lamiraux
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

#ifndef AGIMUS_SOT_CONTACT_ADMITTANCE_HH
#define AGIMUS_SOT_CONTACT_ADMITTANCE_HH

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/feature-abstract.hh>
#include <sot/core/matrix-geometry.hh>

#include <agimus/sot/config.hh>

namespace dynamicgraph {
namespace agimus {

/// Control of the contact of an end-effector by admittance
///
/// This entity performs admittance control of the contact of an end-effector
/// with the environment. It derives from FeatureAbstract. The entity takes as
/// input the error value and Jacobian of another feature controlling the
/// end effector in the absence of contact. When the contact is detected, the
/// value and Jacobian are are replaced by an admittance control regulating
/// the force around a desired value.
///
/// The input signals are
///  \li inputError: error of the feature controlling the end effector when
///      there is no contact,
///  \li inputJacobian: corresponding Jacobian,
///  \li wrench: force and momentum measured by the force sensor,
///  \li ftJacobian: Jacobian of the pose of the force sensor,
///  \li threshold: norm of the force/moment above which the contact is
///      considered as active.
///  \li the desired wrench in the force sensor when there is contact
///  \li the stiffness matrix of the admittance control law.
///
/// The output signals are
/// \li error (from FeatureAbstract)
/// \li jacobian (from FeatureAbstract)
/// \li contact boolean signal informing whether there is contact.
  
using dynamicgraph::sot::FeatureAbstract;
  
class AGIMUS_SOT_DLLAPI ContactAdmittance : public FeatureAbstract
{
 public:
  typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /// Constructor
  ContactAdmittance(const std::string& name);
  virtual unsigned int &getDimension(unsigned int &res, int)
  {
    res=6;
    return res;
  }

 private:
  // Input signals
  // error of the feature controlling the end effector when there is no contact
  SignalPtr<dynamicgraph::Vector, int> errorSIN;
  // corresponding Jacobian
  SignalPtr<dynamicgraph::Matrix, int> jacobianSIN;
  // force and momentum measured by the force sensor,
  SignalPtr<dynamicgraph::Vector, int> wrenchSIN;
  // Jacobian of the pose of the force sensor,
  SignalPtr<dynamicgraph::Matrix, int> ftJacobianSIN;
  // norm of the force/moment above which the contact is considered as active
  SignalPtr<double, int> thresholdSIN;
  // desired wrench
  SignalPtr<dynamicgraph::Vector, int> wrenchDesSIN;
  // stiffness
  SignalPtr<dynamicgraph::Matrix, int> stiffnessSIN;
  // Output signals
  SignalTimeDependent<bool, int> contactSOUT;

   // \brief Compute the error
  virtual dynamicgraph::Vector& computeError(dynamicgraph::Vector &res,
                                             int time);

  // \brief Compute the Jacobian of the error according the robot state.
  virtual dynamicgraph::Matrix& computeJacobian(dynamicgraph::Matrix &res,
                                                int time);

  // Compute whether a contact is detected
  bool& computeContact(bool& res, int time);
  void addCommands();
  DECLARE_NO_REFERENCE;
  }; // class ContactAdmittance
} // namespace agimus
} // namespace dynamicgraph
#endif // AGIMUS_SOT_CONTACT_ADMITTANCE_HH
