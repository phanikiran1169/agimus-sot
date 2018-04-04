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
    static const int BUFFER_SIZE = 50;

    template <typename T>
    struct Add
    {
      void operator () (RosQueuedSubscribe& rosSubscribe,
			const std::string& signal,
			const std::string& topic)
      {
        typedef typename SotToRos<T>::sot_t sot_t;
	typedef typename SotToRos<T>::ros_const_ptr_t ros_const_ptr_t;
        typedef BindedSignal<sot_t, BUFFER_SIZE> BindedSignal_t;
	typedef typename BindedSignal_t::Signal_t Signal_t;

	// Initialize the bindedSignal object.
        BindedSignal_t* bs = new BindedSignal_t(&rosSubscribe);
        SotToRos<T>::setDefault (bs->last);

	// Initialize the signal.
	boost::format signalName ("RosQueuedSubscribe(%1%)::%2%");
	signalName % rosSubscribe.getName () % signal;

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
	  (rosSubscribe.nh ().subscribe (topic, BUFFER_SIZE, callback)); 

	RosQueuedSubscribe::bindedSignal_t bindedSignal (bs);
	rosSubscribe.bindedSignal ()[signal] = bindedSignal;
      }
    };

    // template <typename T, typename R>
    template <typename T, int N>
    template <typename R>
    void BindedSignal<T, N>::writer (const R& data)
    {
      // synchronize with method clear
      wmutex.lock();
      converter (buffer[backIdx], data);
      // assert(!full());
      // No need to synchronize with reader here because:
      // - if the buffer was not empty, then it stays not empty,
      // - if it was empty, then the current value will be used at next time. It
      //   means the transmission bandwidth is too low.
      backIdx = (backIdx+1) % N;
      if (!init) {
        last = buffer[backIdx];
        init = true;
      }
      wmutex.unlock();
    }

    template <typename T, int N>
    T& BindedSignal<T, N>::reader (T& data, int time)
    {
      // synchronize with method clear:
      // If reading from the list cannot be done, then return last value.
      bool readingIsEnabled = rmutex.try_lock();
      if (!readingIsEnabled || entity->readQueue_ == -1 || time < entity->readQueue_) {
        data = last;
      } else {
        if (empty())
          data = last;
        else {
          data = buffer[frontIdx];
          frontIdx = (frontIdx + 1) % N;
          last = data;
        }
      }
      if (readingIsEnabled)
        rmutex.unlock();
      return data;
    }
  } // end of namespace internal.

  template <typename T>
  void RosQueuedSubscribe::add (const std::string& signal, const std::string& topic)
  {
    internal::Add<T> () (*this, signal, topic);
  }
} // end of namespace dynamicgraph.

#endif //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HXX
