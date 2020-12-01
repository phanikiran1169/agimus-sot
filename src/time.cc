#include "time.hh"

namespace dynamicgraph {
namespace agimus {

typedef Time<int> _Time;
template<>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (_Time, "Time");

} // namespace agimus
} // namespace dynamicgraph
