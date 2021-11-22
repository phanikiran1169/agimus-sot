// Copyright 2021 CNRS - Airbus SAS
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
#include "object-localization.hh"

using namespace dynamicgraph::agimus;
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ObjectLocalization, "ObjectLocalization");

typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;



void ObjectLocalization::display(std::ostream& os) const
{
  os << "Object Localization " << getName();
}

void ObjectLocalization::addCommands()
{
  using namespace dynamicgraph::command;
  std::string docstring;
  docstring = "    \n"
    "    Triggers object localization\n"
    "    \n"
    "      Input: time\n"
    "    \n"
    "        sets signal done to False, check signal cMoAvailable, if true\n"
    "        reads and store cMo, set signal done to True.\n";
  addCommand("trigger", makeCommandVoid1(*this, &ObjectLocalization::trigger,
					 docstring));
}

ObjectLocalization::ObjectLocalization(const std::string& name) :
  Entity(name),
  cMoMeasuredSIN
  (0x0, "ObjectLocalization("+name+")::input(MatrixHomo)::cMoMeasured"),
  cMoAvailableSIN(0x0,
		  "ObjectLocalization("+name+")::input(bool)::cMoAvailable"),
  wMcSIN(0x0, "ObjectLocalization("+name+")::input(MatrixHomo)::wMc"),
  wMoSOUT("ObjectLocalization("+name+")::output(MatrixHomo)::wMo"),
  cMoSOUT(boost::bind(&ObjectLocalization::compute_cMo, this, _1, _2),
	  wMcSIN,
	  "ObjectLocalization("+name+")::output(MatrixHomo)::cMo"),
  doneSOUT("ObjectLocalization("+name+")::output(bool)::done")
{
  addCommands();
  doneSOUT.setConstant(false);
  Entity::signalRegistration(cMoSOUT << wMcSIN);
  Entity::signalRegistration(cMoMeasuredSIN);
  Entity::signalRegistration(cMoAvailableSIN);
  Entity::signalRegistration(wMoSOUT);
  Entity::signalRegistration(doneSOUT);
}

void ObjectLocalization::trigger(const int& t)
{
  doneSOUT.setConstant(false);
  cMoAvailableSIN.recompute(t);
  if (cMoAvailableSIN.access(t)){
    MatrixHomogeneous cMo, wMc, wMo;
    cMoMeasuredSIN.recompute(t);
    wMcSIN.recompute(t);
    cMo = cMoMeasuredSIN.access(t);
    wMc = wMcSIN.access(t);
    wMo = wMc*cMo;
    wMoSOUT.setConstant(wMo);
    doneSOUT.setConstant(true);
  }
}

MatrixHomogeneous& ObjectLocalization::compute_cMo
(MatrixHomogeneous& res, int t)
{
  wMcSIN.recompute(t);
  MatrixHomogeneous cMw = wMcSIN.access(t).inverse();
  res = cMw * wMoSOUT.access(t);
  return res;
}
