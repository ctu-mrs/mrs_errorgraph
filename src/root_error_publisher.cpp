/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <mrs_errorgraph/errorgraph.h>

#include <mrs_errorgraph/ErrorgraphElementArray.h>

//}

class RootErrorPublisher
{
  public:
    using errorgraph_element_msg_t = mrs_errorgraph::errorgraph_element_msg_t;
    using errorgraph_element_array_msg_t = mrs_errorgraph::ErrorgraphElementArray;

    RootErrorPublisher(ros::NodeHandle& nh)
    {
      /* obtain node handle */
      nh_ = nh;

      /* waits for the ROS to publish clock */
      ros::Time::waitForValid();

      /* load parameters */
      mrs_lib::ParamLoader param_loader(nh_, "RootErrorPublisher");

      param_loader.addYamlFileFromParam("config");

      const auto main_timer_rate = param_loader.loadParam2<double>("main_timer_rate");

      if (!param_loader.loadedSuccessfully())
      {
        ROS_ERROR("[RootErrorPublisher]: Could not load all parameters!");
        ros::shutdown();
      }

      // | ----------------------- publishers ----------------------- |
      pub_root_errors_ = nh_.advertise<errorgraph_element_array_msg_t>("out/root_errors", 1);

      // | ----------------------- subscribers ---------------------- |

      tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(nh_, ros::Rate(1.0));
      mrs_lib::SubscribeHandlerOptions shopts;
      shopts.nh = nh_;
      shopts.node_name = "RootErrorPublisher";
      shopts.no_message_timeout = ros::Duration(5.0);
      shopts.timeout_manager = tim_mgr_;
      shopts.threadsafe = true;
      shopts.autostart = true;
      shopts.queue_size = 10;
      shopts.transport_hints = ros::TransportHints().unreliable();

      // | --------------------- Main subscriber -------------------- |
      sh_errorgraph_error_msg_ = mrs_lib::SubscribeHandler<errorgraph_element_msg_t>(shopts, "in/errors", &RootErrorPublisher::cbkElement, this);

      // | ------------------------- timers ------------------------- |

      timer_main_ = nh_.createTimer(ros::Rate(main_timer_rate), &RootErrorPublisher::timerMain, this);

      // | --------------------- finish the init -------------------- |

      ROS_INFO("[RootErrorPublisher]: initialized");
      ROS_INFO("[RootErrorPublisher]: --------------------");
    }
  private:
    ros::NodeHandle nh_;

    std::mutex errorgraph_mtx_;
    mrs_errorgraph::Errorgraph errorgraph_;

    // | --------------------- ROS publishers --------------------- |

    ros::Publisher pub_root_errors_;

    // | ---------------------- ROS subscribers --------------------- |

    std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

    mrs_lib::SubscribeHandler<errorgraph_element_msg_t> sh_errorgraph_error_msg_;

    // | ----------------------- main timer ----------------------- |

    ros::Timer timer_main_;
    void timerMain(const ros::TimerEvent& event)
    {
      std::scoped_lock lck(errorgraph_mtx_);

      const auto now = ros::Time::now();
      const auto roots = errorgraph_.find_error_roots();
      errorgraph_element_array_msg_t msg;
      msg.stamp = now;
      for (const auto& root_element : roots)
        msg.elements.push_back(root_element->to_msg());
      pub_root_errors_.publish(msg);
    }

    // | ----------------- error message callback ----------------- |
    void cbkElement(const errorgraph_element_msg_t::ConstPtr element_msg)
    {
      std::scoped_lock lck(errorgraph_mtx_);
      errorgraph_.add_element_from_msg(*element_msg);
    }
};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "root_error_publisher");
  ros::NodeHandle nh("~");

  RootErrorPublisher rep(nh);
  ros::spin();
}
