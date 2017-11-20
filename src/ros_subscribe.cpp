#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <dynamic-graph/factory.h>

#include "dynamic_graph_bridge/ros_init.hh"
#include "ros_subscribe.hh"

namespace dynamicgraph
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosQueuedSubscribe, "RosQueuedSubscribe");

  namespace command
  {
    namespace rosSubscribe
    {
      Clear::Clear
      (RosQueuedSubscribe& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value Clear::doExecute ()
      {
	RosQueuedSubscribe& entity =
	  static_cast<RosQueuedSubscribe&> (owner ());

	entity.clear ();
	return Value ();
      }

      ClearQueue::ClearQueue
      (RosQueuedSubscribe& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of (Value::STRING),
	   docstring)
      {}

      Value ClearQueue::doExecute ()
      {
	RosQueuedSubscribe& entity =
	  static_cast<RosQueuedSubscribe&> (owner ());

	std::vector<Value> values = getParameterValues ();
	const std::string& signal = values[0].value ();
        entity.clearQueue (signal);

	return Value ();
      }

      List::List
      (RosQueuedSubscribe& entity, const std::string& docstring)
	: Command
	  (entity,
	   std::vector<Value::Type> (),
	   docstring)
      {}

      Value List::doExecute ()
      {
	RosQueuedSubscribe& entity =
	  static_cast<RosQueuedSubscribe&> (owner ());
	return Value (entity.list ());
      }

      Add::Add
      (RosQueuedSubscribe& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of
	   (Value::STRING) (Value::STRING) (Value::STRING),
	   docstring)
      {}

      Value Add::doExecute ()
      {
	RosQueuedSubscribe& entity =
	  static_cast<RosQueuedSubscribe&> (owner ());
	std::vector<Value> values = getParameterValues ();

	const std::string& type = values[0].value ();
	const std::string& signal = values[1].value ();
	const std::string& topic = values[2].value ();

	if (type == "double")
	  entity.add<double> (signal, topic);
	else if (type == "unsigned")
	  entity.add<unsigned int> (signal, topic);
	else if (type == "matrix")
	  entity.add<dg::Matrix> (signal, topic);
	else if (type == "vector")
	  entity.add<dg::Vector> (signal, topic);
	else if (type == "vector3")
	  entity.add<specific::Vector3> (signal, topic);
	else if (type == "matrixHomo")
	  entity.add<sot::MatrixHomogeneous> (signal, topic);
	else if (type == "twist")
	  entity.add<specific::Twist> (signal, topic);
	else
	  throw std::runtime_error("bad type");
	return Value ();
      }

      Rm::Rm
      (RosQueuedSubscribe& entity, const std::string& docstring)
	: Command
	  (entity,
	   boost::assign::list_of (Value::STRING),
	   docstring)
      {}

      Value Rm::doExecute ()
      {
	RosQueuedSubscribe& entity =
	  static_cast<RosQueuedSubscribe&> (owner ());
	std::vector<Value> values = getParameterValues ();
	const std::string& signal = values[0].value ();
	entity.rm (signal);
	return Value ();
      }
    } // end of errorEstimator.
  } // end of namespace command.

  const std::string RosQueuedSubscribe::docstring_
  ("Subscribe to a ROS topics and convert it into a dynamic-graph signals.\n"
   "\n"
   "  Use command \"add\" to subscribe to a new signal.\n");

  RosQueuedSubscribe::RosQueuedSubscribe (const std::string& n)
    : dynamicgraph::Entity(n),
      nh_ (rosInit (true)),
      bindedSignal_ ()
  {
    std::string docstring =
      "\n"
      "  Add a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - type: string among ['double', 'matrix', 'vector', 'vector3',\n"
      "                          'matrixHomo', 'twist'],\n"
      "    - signal: the signal name in dynamic-graph,\n"
      "    - topic:  the topic name in ROS.\n"
      "\n";
    addCommand ("add",
		new command::rosSubscribe::Add
		(*this, docstring));
    docstring =
      "\n"
      "  Remove a signal reading data from a ROS topic\n"
      "\n"
      "  Input:\n"
      "    - name of the signal to remove (see method list for the list of signals).\n"
      "\n";
    addCommand ("rm",
		new command::rosSubscribe::Rm
		(*this, docstring));
    docstring =
      "\n"
      "  Remove all signals reading data from a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
    addCommand ("clear",
		new command::rosSubscribe::Clear
		(*this, docstring));
    docstring =
      "\n"
      "  List signals reading data from a ROS topic\n"
      "\n"
      "  No input:\n"
      "\n";
    addCommand ("list",
		new command::rosSubscribe::List
		(*this, docstring));
    docstring =
      "\n"
      "  Empty the queue of a given signal\n"
      "\n"
      "  No input:\n"
      "    - name of the signal (see method list for the list of signals).\n"
      "\n";
    addCommand ("clearQueue",
		new command::rosSubscribe::ClearQueue
		(*this, docstring));
  }

  RosQueuedSubscribe::~RosQueuedSubscribe ()
  {
    std::cout << getName() << ": Delete" << std::endl;
  }

  void RosQueuedSubscribe::display (std::ostream& os) const
  {
    os << CLASS_NAME << std::endl;
  }

  void RosQueuedSubscribe::rm (const std::string& signal)
  {
    std::string signalTs = signal+"Timestamp";

    signalDeregistration(signal);
    bindedSignal_.erase (signal);

    if(bindedSignal_.find(signalTs) != bindedSignal_.end())
    {
       signalDeregistration(signalTs);
       bindedSignal_.erase(signalTs);
    }
  }

  std::string RosQueuedSubscribe::list ()
  {
    std::string result("[");
    for (std::map<std::string, bindedSignal_t>::const_iterator it =
	   bindedSignal_.begin (); it != bindedSignal_.end (); it++) {
      result += "'" + it->first + "',";
    }
    result += "]";
    return result;
  }

  void RosQueuedSubscribe::clear ()
  {
    std::map<std::string, bindedSignal_t>::iterator it = bindedSignal_.begin();
    for(; it!= bindedSignal_.end(); )
    {
      rm(it->first);
      it = bindedSignal_.begin();
    }
  }

  void RosQueuedSubscribe::clearQueue (const std::string& signal)
  {
    if(bindedSignal_.find(signal) != bindedSignal_.end())
    {
       bindedSignal_[signal]->clear();
    }
  }

  std::string RosQueuedSubscribe::getDocString () const
  {
    return docstring_;
  }
} // end of namespace dynamicgraph.
