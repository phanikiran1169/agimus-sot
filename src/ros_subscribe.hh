#ifndef DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
# define DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
# include <iostream>
# include <map>

# include <boost/shared_ptr.hpp>
# include <boost/thread/mutex.hpp>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/command.h>
# include <sot/core/matrix-geometry.hh>

# include <ros/ros.h>

# include "converter.hh"
# include "sot_to_ros.hh"

namespace dynamicgraph
{
  class RosQueuedSubscribe;

  namespace command
  {
    namespace rosSubscribe
    {
      using ::dynamicgraph::command::Command;
      using ::dynamicgraph::command::Value;

# define ROS_SUBSCRIBE_MAKE_COMMAND(CMD)			\
      class CMD : public Command			\
      {							\
      public:						\
	CMD (RosQueuedSubscribe& entity,				\
	     const std::string& docstring);		\
	virtual Value doExecute ();			\
      }

      ROS_SUBSCRIBE_MAKE_COMMAND(Add);
      ROS_SUBSCRIBE_MAKE_COMMAND(Clear);
      ROS_SUBSCRIBE_MAKE_COMMAND(List);
      ROS_SUBSCRIBE_MAKE_COMMAND(Rm);
      ROS_SUBSCRIBE_MAKE_COMMAND(ClearQueue);
      ROS_SUBSCRIBE_MAKE_COMMAND(QueueSize);
      ROS_SUBSCRIBE_MAKE_COMMAND(ReadQueue);

#undef ROS_SUBSCRIBE_MAKE_COMMAND

    } // end of namespace errorEstimator.
  } // end of namespace command.

  class RosQueuedSubscribe;

  namespace internal
  {
    template <typename T>
    struct Add;

    struct BindedSignalBase {
      typedef boost::shared_ptr<ros::Subscriber> Subscriber_t;

      BindedSignalBase(RosQueuedSubscribe* e) : entity(e) {}
      virtual ~BindedSignalBase() {}

      virtual void clear () = 0;
      virtual std::size_t size () const = 0;

      Subscriber_t subscriber;
      RosQueuedSubscribe* entity;
    };

    template <typename T>
    struct BindedSignal : BindedSignalBase {
      typedef dynamicgraph::Signal<T, int> Signal_t;
      typedef boost::shared_ptr<Signal_t> SignalPtr_t;
      typedef std::queue<T> Queue_t;

      BindedSignal(RosQueuedSubscribe* e) : BindedSignalBase (e), init(false) {}
      ~BindedSignal()
      {
        std::cout << signal->getName() << ": Delete" << std::endl;
        signal.reset();
        clear();
      }

      void clear ()
      {
        qmutex.lock();
        queue = Queue_t();
        qmutex.unlock();
      }

      std::size_t size () const
      {
        return queue.size();
      }

      SignalPtr_t signal;
      Queue_t queue;
      boost::mutex qmutex;
      T last;
      bool init;

      template <typename R> void writer (const R& data);
      T& reader (T& val, int time);
    };
  } // end of internal namespace.


  /// \brief Publish ROS information in the dynamic-graph.
  class RosQueuedSubscribe : public dynamicgraph::Entity
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
    typedef boost::posix_time::ptime ptime;
  public:
    typedef boost::shared_ptr<internal::BindedSignalBase> bindedSignal_t;

    RosQueuedSubscribe (const std::string& n);
    virtual ~RosQueuedSubscribe ();

    virtual std::string getDocString () const;
    void display (std::ostream& os) const;

    void add (const std::string& signal, const std::string& topic);
    void rm (const std::string& signal);
    std::string list ();
    void clear ();
    void clearQueue (const std::string& signal);
    void readQueue (bool read);
    std::size_t queueSize (const std::string& signal) const;

    template <typename T>
    void add (const std::string& signal, const std::string& topic);

    std::map<std::string, bindedSignal_t>&
    bindedSignal ()
    {
      return bindedSignal_;
    }

    ros::NodeHandle& nh ()
    {
      return nh_;
    }

    template <typename R, typename S>
    void callback
    (boost::shared_ptr<dynamicgraph::SignalPtr<S, int> > signal,
     const R& data);

    template <typename R>
    void callbackTimestamp
    (boost::shared_ptr<dynamicgraph::SignalPtr<ptime, int> > signal,
     const R& data);

    template <typename T>
    friend class internal::Add;
  private:
    static const std::string docstring_;
    ros::NodeHandle& nh_;
    std::map<std::string, bindedSignal_t> bindedSignal_;

    bool readQueue_;
    // Signal<bool, int> readQueue_;

    template <typename T>
    friend class internal::BindedSignal;
  };
} // end of namespace dynamicgraph.

# include "ros_subscribe.hxx"
#endif //! DYNAMIC_GRAPH_ROS_SUBSCRIBE_HH
