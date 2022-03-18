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
typedef dynamicgraph::Matrix matrix_t;
typedef dynamicgraph::Vector vector_t;


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
              errorSIN << thresholdSIN << wrenchSIN << stiffnessSIN <<
              ftJacobianSIN << jacobianSIN,
              "ContactAdmittance("+name+")::output(int)::contact")
{
  addCommands();
  signalRegistration(errorSIN << jacobianSIN << wrenchSIN << ftJacobianSIN <<
                     contactSOUT << thresholdSIN << wrenchDesSIN <<
                     stiffnessSIN);
}

vector_t& ContactAdmittance::computeError
(vector_t &res, int time)
{
  ContactType contact((ContactType)contactSOUT(time));
  if (contact == NO_CONTACT || contact == CONTACT_RELEASED){
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
  int contact(contactSOUT(time));
  if (contact == NO_CONTACT || contact == CONTACT_RELEASED){
    res = jacobianSIN(time);
  } else {
    res = -stiffnessSIN(time) * ftJacobianSIN(time);
  }
  return res;
}

int& ContactAdmittance::computeContact(int& res, int time)
{
  vector_t wrench(wrenchSIN(time));

  // If norm of wrench is below the threshold, there is no contact
  if (wrench.norm() < thresholdSIN(time)){
    res = (int)NO_CONTACT;
    return res;
  }
  matrix_t JinPinv = jacobianSIN(time).completeOrthogonalDecomposition().
    pseudoInverse();
  matrix_t a = wrenchSIN(time).transpose()*stiffnessSIN(time)*
    ftJacobianSIN(time)*JinPinv*errorSIN(time);
  if (a(0,0) < 0) {
    res = (int)CONTACT_RELEASED;
  } else {
    res = (int)ACTIVE_CONTACT;
  }
  return res;
}

} // namespace agimus
} // namespace dynamicgraph
