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
#include <dynamic-graph/all-commands.h>
#include "contact-admittance.hh"

namespace dynamicgraph {
namespace agimus {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ContactAdmittance, "ContactAdmittance");

typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;
typedef dynamicgraph::Vector vector_t;


void ContactAdmittance::display(std::ostream& os) const
{
  os << "ContactAdmittance " << getName();
}

void ContactAdmittance::addCommands()
{
  std::string docstring;
  docstring=
    "    \n"
    "    Set wrench offset from current wrench input signal\n"
    "    \n"
    "      Input:\n"
    "        time: time to recompute the wrench signal\n"
    "    \n";


  addCommand("getWrenchOffset", command::makeCommandVoid1
    (*this, &ContactAdmittance::getWrenchOffset, docstring));

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
  wrenchOffsetSOUT("ContactAdmittance("+name+")::output(vector)::wrenchOffset"),
  contactSOUT(boost::bind(&ContactAdmittance::computeContact, this, _1, _2),
              errorSIN << thresholdSIN << wrenchSIN << stiffnessSIN <<
              ftJacobianSIN << jacobianSIN,
              "ContactAdmittance("+name+")::output(int)::contact"),
  releaseCriterionSOUT("ContactAdmittance("+name+")::output(double)::releaseCriterion"),
  wrenchMinusOffsetSOUT("ContactAdmittance("+name+")::output(vector)::wrenchMinusOffset"),
  contactCounter_(0), nContactIterations_(5), contactState_(NO_CONTACT)
{
  addCommands();
  signalRegistration(errorSIN << jacobianSIN << wrenchSIN << ftJacobianSIN <<
                     contactSOUT << thresholdSIN << wrenchDesSIN <<
                     stiffnessSIN);
  signalRegistration(wrenchOffsetSOUT);
  signalRegistration(releaseCriterionSOUT);
  signalRegistration(wrenchMinusOffsetSOUT);
  dynamicgraph::Vector offset(6); offset.setZero();
  wrenchOffsetSOUT.setConstant(offset);
}

void ContactAdmittance::getWrenchOffset(const int& time)
{
  wrenchOffsetSOUT.setConstant(wrenchSIN(time));
  // Reset contact state since norm of force may have been above the threshold
  // before this initialization.
  contactState_ = NO_CONTACT;
}

vector_t& ContactAdmittance::computeError
(vector_t &res, int time)
{
  ContactState contact((ContactState)contactSOUT(time));
  if (contact == NO_CONTACT || contact == GOING_TO_CONTACT){
    res = errorSIN(time);
  } else {
    // in case of contact, the error is the difference between the measured
    // and desired wrenchs
    res = wrenchSIN(time) - wrenchOffsetSOUT(time) - wrenchDesSIN(time);
  }
  wrenchMinusOffsetSOUT.setConstant(wrenchSIN(time) - wrenchOffsetSOUT(time));
  return res;
}

dynamicgraph::Matrix& ContactAdmittance::computeJacobian
(dynamicgraph::Matrix &res, int time)
{
  int contact(contactSOUT(time));
  if (contact == NO_CONTACT || contact == GOING_TO_CONTACT){
    res = jacobianSIN(time);
  } else {
    res = -stiffnessSIN(time) * ftJacobianSIN(time);
  }
  return res;
}

int& ContactAdmittance::computeContact(int& res, int time)
{
  vector_t w(wrenchSIN(time)-wrenchOffsetSOUT(time));
  // Compute state of contact
  bool forceAboveThreshold(w.head<3>().norm() >=
			   thresholdSIN(time));
  releaseCriterionSOUT.setConstant(0);
  switch(contactState_){
  case NO_CONTACT:
    if (forceAboveThreshold) {
      // first detection, switch to next state and increase counter
      contactCounter_=1;
      res = GOING_TO_CONTACT;
    } else{
      // otherwise remain in state NO_CONTACT
      res = NO_CONTACT;
    }
    break;
  case GOING_TO_CONTACT:
    if (forceAboveThreshold) {
      // increase counter
      ++contactCounter_;
      if (contactCounter_ >= nContactIterations_){
        // if number of iteration reached, switch to ACTIVE_CONTACT.
        res = ACTIVE_CONTACT;
        contactCounter_ = 0;
      } else{
        res = GOING_TO_CONTACT;
      }
     } else{
      // Return to state NO_CONTACT
      res = NO_CONTACT;
      contactCounter_ = 0;
     }
    break;
  case ACTIVE_CONTACT:
    JinPinv_ = jacobianSIN(time).completeOrthogonalDecomposition().
      pseudoInverse();
    a_ = w.transpose()*stiffnessSIN(time)*ftJacobianSIN(time)*
      JinPinv_*errorSIN(time);
    releaseCriterionSOUT.setConstant(a_(0,0));
    if (a_(0,0) >= 0) {
      // Stay in this state
      res = ACTIVE_CONTACT;
    } else{
      // Go to state RELEASING_CONTACT
      res = RELEASING_CONTACT;
    }
    break;
  case RELEASING_CONTACT:
    JinPinv_ = jacobianSIN(time).completeOrthogonalDecomposition().
      pseudoInverse();
    a_ = w.transpose()*stiffnessSIN(time)*ftJacobianSIN(time)*
      JinPinv_*errorSIN(time);
    releaseCriterionSOUT.setConstant(a_(0,0));
    if (a_(0,0) >= 0) {
      // Return to state ACTIVE_CONTACT
      res = ACTIVE_CONTACT;
      contactCounter_ = 0;
    } else{
      // increase counter
      ++contactCounter_;
      if (contactCounter_ >= nContactIterations_){
        // if number of iteration reached, switch to NO_CONTACT.
        res = NO_CONTACT;
        contactCounter_ = 0;
      } else{
        res = RELEASING_CONTACT;
      }
    }
    break;
  };
  contactState_ = static_cast<ContactState>(res);
  return res;
}

} // namespace agimus
} // namespace dynamicgraph
