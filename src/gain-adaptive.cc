#include "gain-adaptive.hh"

#include <sot/core/debug.hh>
#include <sot/core/exception-signal.hh>
#include <sot/core/factory.hh>

#include <sot/core/debug.hh>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>

using namespace dynamicgraph::agimus;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SafeGainAdaptive, "SafeGainAdaptive");

void SafeGainAdaptive::addCommands()
{
  using namespace ::dynamicgraph::command;
  std::string docstring;
  docstring = "    \n"
              "    Set the parameters\n"
              "      Input:\n"
              "        floating point value: a,\n"
              "        floating point value: b,\n"
              "        floating point value: c,\n"
              "        floating point value: d,\n"
              "    \n";
  addCommand("setParameters", makeCommandVoid4(*this, &SafeGainAdaptive::setParameters, docstring));
  docstring = "    \n"
              "    Compute the parameters\n"
              "      Input:\n"
              "        floating point value: value at 0,\n"
              "        floating point value: value of g(e) * e at when e goes to infinity,\n"
              "        floating point value: value of e which gives the maximum of the exponential part.\n"
              "        floating point value: value of e for which the hyperbolic part is half of its final value.\n"
              "    \n";
  addCommand("computeParameters", makeCommandVoid4(*this, &SafeGainAdaptive::computeParameters, docstring));

  addCommand("getCoeffs", makeDirectGetter(*this, &cs, docDirectGetter("coefficients", "vector of size 4")));
}

SafeGainAdaptive::SafeGainAdaptive(const std::string &name)
  : Entity(name), cs (4),
  errorSIN(NULL, "SafeGainAdaptive(" + name + ")::input(vector)::error"),
  gainSOUT(boost::bind(&SafeGainAdaptive::computeGain, this, _1, _2),
           errorSIN,
           "SafeGainAdaptive(" + name + ")::output(double)::gain")
{
  computeParameters(0.1, 1, 0.1, 2.);
  Entity::signalRegistration(gainSOUT << errorSIN);
  addCommands();
}

void SafeGainAdaptive::setParameters(const double &a, const double &b,
    const double &c, const double &d)
{
  cs << a, b, c, d;
}

void SafeGainAdaptive::computeParameters(
    const double &valueAt0,
    const double &valueAtInfty,
    const double &errNormOfMaxExp,
    const double &errNormOfHalfHyp)
{
  double a, b, c, d;
  c = valueAtInfty;
  if (errNormOfMaxExp <= 0) b = 0;
  else b = 1 / errNormOfMaxExp;

  d = atanh(.5) / errNormOfHalfHyp;
  a = valueAt0 - c * d;

  setParameters (a, b, c, d);
}

void SafeGainAdaptive::display(std::ostream &os) const
{
  os << "Gain Adaptative " << getName();
  try {
    os << " = " << double(gainSOUT.accessCopy());
  } catch (ExceptionSignal e) {
  }
  os << " (" << cs.transpose() << ") ";
}

double &SafeGainAdaptive::computeGain(double &res, int t)
{
  const dynamicgraph::Vector &error = errorSIN(t);
  const double norm = error.norm();
  const double& a = cs[0];
  const double& b = cs[1];
  const double& c = cs[2];
  const double& d = cs[3];

  if (norm < 1e-6) {
    res = (a+c*d) - a*b*norm
      // + (a*b*b/2 - c*d*d*d/3)*norm*norm
      ;
  } else {
    res = a * std::exp(-b*norm) + c * std::tanh(d*norm) / norm;
  }
  return res;
}
