#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
# define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
# include <vector>
# include <boost/bind.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <dynamic-graph/signal-caster.h>
# include <dynamic-graph/linear-algebra.h>
# include <dynamic-graph/signal-cast-helper.h>
# include <std_msgs/Float64.h>
# include "dynamic_graph_bridge_msgs/Matrix.h"
# include "dynamic_graph_bridge_msgs/Vector.h"

namespace dg = dynamicgraph;

namespace dynamicgraph
{
  namespace internal
  {
    template <typename T>
    struct Add
    {
      void operator () (RosQueuedSubscribe& rosSubscribe,
			const std::string& type,
			const std::string& signal,
			const std::string& topic)
      {
        typedef typename SotToRos<T>::sot_t sot_t;
	typedef typename SotToRos<T>::ros_const_ptr_t ros_const_ptr_t;
        typedef BindedSignal<sot_t> BindedSignal_t;
	typedef typename BindedSignal_t::Signal_t Signal_t;

	// Initialize the bindedSignal object.
        BindedSignal_t* bs = new BindedSignal_t(&rosSubscribe);
        SotToRos<T>::setDefault (bs->last);

	// Initialize the signal.
	boost::format signalName ("RosQueuedSubscribe(%1%)::output(%2%)::%3%");
	signalName % rosSubscribe.getName () % type % signal;

	bs->signal.reset (new Signal_t (signalName.str ()));
        bs->signal->setFunction (boost::bind(&BindedSignal_t::reader, bs, _1, _2));
	rosSubscribe.signalRegistration (*bs->signal);

	// Initialize the subscriber.
	typedef boost::function<void (const ros_const_ptr_t& data)> callback_t;
	callback_t callback = boost::bind
	  (&BindedSignal_t::template writer<ros_const_ptr_t>, bs, _1);

  // Keep 50 messages in queue, but only 20 are sent every 100ms
  // -> No message should be lost because of a full buffer
	bs->subscriber =
	  boost::make_shared<ros::Subscriber>
	  (rosSubscribe.nh ().subscribe (topic, 50, callback)); 

	RosQueuedSubscribe::bindedSignal_t bindedSignal (bs);
	rosSubscribe.bindedSignal ()[signal] = bindedSignal;
      }
    };

    // template <typename T, typename R>
    template <typename T>
    template <typename R>
    void BindedSignal<T>::writer (const R& data)
    {
      T value;
      converter (value, data);
      if (!init) {
        last = value;
        init = true;
      }
      qmutex.lock();
      queue.push (value);
      qmutex.unlock();
    }

    template <typename T>
    T& BindedSignal<T>::reader (T& data, int time)
    {
      if (entity->readQueue_ == -1 || time < entity->readQueue_) {
        data = last;
      } else {
        qmutex.lock();
        if (queue.empty())
          data = last;
        else {
          data = queue.front();
          queue.pop();
          last = data;
        }
        qmutex.unlock();
      }
      return data;
    }
  } // end of namespace internal.

  template <typename T>
  void RosQueuedSubscribe::add (const std::string& type, const std::string& signal, const std::string& topic)
  {
    internal::Add<T> () (*this, type, signal, topic);
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
