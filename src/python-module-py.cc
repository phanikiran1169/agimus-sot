#include "dynamic-graph/python/module.hh"

#include "time.hh"
#include "contact-admittance.hh"
#include "gain-adaptive.hh"
#include "holonomic-constraint.hh"
#include "object-localization.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");
  dg::python::exposeEntity<dg::agimus::Time<int> >();
  dg::python::exposeEntity<dg::agimus::ContactAdmittance>();
  dg::python::exposeEntity<dg::agimus::SafeGainAdaptive>();
  dg::python::exposeEntity<dg::agimus::HolonomicConstraint>();
  dg::python::exposeEntity<dg::agimus::ObjectLocalization>();
}
