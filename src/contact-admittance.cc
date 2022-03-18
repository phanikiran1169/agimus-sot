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

#include <dynamic-graph/factory.h>
#include "contact-admittance.hh"

namespace dynamicgraph {
namespace agimus {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ContactAdmittance, "ContactAdmittance");

typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;



void ContactAdmittance::display(std::ostream& os) const
{
  os << "ContactAdmittance " << getName();
}

void ContactAdmittance::addCommands()
{
}

ContactAdmittance::ContactAdmittance(const std::string& name) :
  FeatureAbstract(name),
  errorSIN(0x0, "ContactAdmittance("+name+")::input(vector)::errorIn"),
  jacobianSIN(0x0, "ContactAdmittance("+name+")::input(matrix)::jacobianIn"),
  wrenchSIN(0x0, "ContactAdmittance("+name+")::input(vector)::wrench"),
  ftJacobianSIN(0x0, "ContactAdmittance("+name+")::input(matrix)::ftJacobian"),
  thresholdSIN(0x0, "ContactAdmittance("+name+")::input(double)::threshold"),
  wrenchDesSIN(0x0, "ContactAdmittance("+name+")::input(vector)::wrenchDes"),
  stiffnessSIN(0x0, "ContactAdmittance("+name+")::input(matrix)::stiffness"),
  contactSOUT(boost::bind(&ContactAdmittance::computeContact, this, _1, _2),
              errorSIN << thresholdSIN,
              "ContactAdmittance("+name+")::output(bool)::contact")
{
  addCommands();
  signalRegistration(errorSIN << jacobianSIN << wrenchSIN << ftJacobianSIN <<
                     contactSOUT << thresholdSIN << wrenchDesSIN <<
                     stiffnessSIN);
}

dynamicgraph::Vector& ContactAdmittance::computeError
(dynamicgraph::Vector &res, int time)
{
  bool contact(contactSOUT(time));
  if (contact){
    res = errorSIN(time);
  } else {
    // in case of contact, the error is the difference between the measured
    // and desired wrenchs
    res = wrenchSIN(time) - wrenchDesSIN(time);
  }
  return res;
}

dynamicgraph::Matrix& ContactAdmittance::computeJacobian
(dynamicgraph::Matrix &res, int time)
{
  bool contact(contactSOUT(time));
  if (contact){
    res = jacobianSIN(time);
  } else {
    res = -stiffnessSIN(time) * ftJacobianSIN(time);
  }
  return res;
}

bool& ContactAdmittance::computeContact(bool& res, int time)
{
  dynamicgraph::Vector wrench(wrenchSIN(time));
  res = (wrench.norm() >= thresholdSIN(time));
  return res;
}

} // namespace agimus
} // namespace dynamicgraph
