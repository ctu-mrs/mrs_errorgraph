#pragma once

#include <ros/ros.h>

#include <mrs_errorgraph/ErrorgraphElement.h>

namespace mrs_errorgraph
{

  class ErrorPublisher
  {
    public:
      using error_id_t = uint16_t;

    private:
      struct error_wrapper_t
      {
        std::optional<error_id_t> id; 
        mrs_errorgraph::ErrorgraphError msg;
      };

    public:

      ErrorPublisher(const ros::NodeHandle& nh, const std::string& node_name, const std::string& component_name, const ros::Rate& publish_period = ros::Rate(1.0))
        : nh_(nh), node_name_(node_name), component_name_(component_name)
      {
        pub_ = nh_.advertise<mrs_errorgraph::ErrorgraphElement>("errors", 1, true);
        tim_publish_ = nh_.createTimer(publish_period, &ErrorPublisher::timPublish, this);
      };

      void flush()
      {
        publishErrors();
      }

      void flushAndShutdown()
      {
        publishErrors();
        tim_publish_ = {};
        ros::Duration(1.0).sleep();
        ros::shutdown();
      }

      template <typename enum_T>
      void addGeneralError(const enum_T id, const std::string& description)
      {
        addGeneralError(static_cast<error_id_t>(id), description);
      }

      void addGeneralError(const error_id_t id, const std::string& description)
      {
        const auto now = ros::Time::now();
        std::scoped_lock lck(errors_mtx_);
        for (auto& error_wrapper : errors_)
        {
          if (error_wrapper.id.has_value() && error_wrapper.id.value() == id)
          {
            error_wrapper.msg.type = description;
            error_wrapper.msg.stamp = now;
            return;
          }
        }
        mrs_errorgraph::ErrorgraphError msg;
        msg.type = description;
        msg.stamp = now;
        errors_.push_back({id, std::move(msg)});
      }

      void addOneshotError(const std::string& description)
      {
        const auto now = ros::Time::now();
        std::scoped_lock lck(errors_mtx_);
        mrs_errorgraph::ErrorgraphError msg;
        msg.type = description;
        msg.stamp = now;
        errors_.push_back({std::nullopt, std::move(msg)});
      }

      void addWaitingForNodeError(const node_id& node_id)
      {
        const auto now = ros::Time::now();
        std::scoped_lock lck(errors_mtx_);
        for (auto& error_wrapper : errors_)
        {
          if (error_wrapper.msg.type == mrs_errorgraph::ErrorgraphError::TYPE_WAITING_FOR_NODE
           && error_wrapper.msg.waited_for_node.node == node_id.node
           && error_wrapper.msg.waited_for_node.component == node_id.component)
          {
            error_wrapper.msg.stamp = now;
            return;
          }
        }
        mrs_errorgraph::ErrorgraphError msg;
        msg.type = mrs_errorgraph::ErrorgraphError::TYPE_WAITING_FOR_NODE;
        msg.stamp = now;
        msg.waited_for_node.node = node_id.node;
        msg.waited_for_node.component = node_id.component;
        errors_.push_back({std::nullopt, std::move(msg)});
      }

    private:
      ros::NodeHandle nh_;
      std::string node_name_;
      std::string component_name_;

      std::mutex errors_mtx_;
      std::vector<error_wrapper_t> errors_;

      std::mutex pub_mtx_;
      ros::Publisher pub_;

      ros::Timer tim_publish_;

      void timPublish([[maybe_unused]] const ros::TimerEvent& evt)
      {
        publishErrors();
      }

      void publishErrors()
      {
        std::scoped_lock lck(errors_mtx_, pub_mtx_);
        mrs_errorgraph::ErrorgraphElement msg;
        msg.stamp = ros::Time::now();
        msg.source_node.node = node_name_;
        msg.source_node.component = component_name_;
        for (const auto& error_wrapper : errors_)
          msg.errors.emplace_back(error_wrapper.msg);
        pub_.publish(msg);
        errors_.clear();
      }
  };

}
