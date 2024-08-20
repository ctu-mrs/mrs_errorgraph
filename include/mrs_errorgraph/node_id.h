#pragma once
#include <mrs_errorgraph/ErrorgraphNodeID.h>

namespace mrs_errorgraph
{
  using errorgraph_node_id_msg_t = mrs_errorgraph::ErrorgraphNodeID;

  struct node_id_t
  {
    std::string node;
    std::string component;

    inline bool operator==(const node_id_t& other) const
    {
      return node == other.node && component == other.component;
    }

    static inline node_id_t from_msg(const errorgraph_node_id_msg_t& msg)
    {
      node_id_t ret;
      ret.node = msg.node;
      ret.component = msg.component;
      return ret;
    }

    inline errorgraph_node_id_msg_t to_msg() const
    {
      errorgraph_node_id_msg_t ret;
      ret.node = node;
      ret.component = component;
      return ret;
    }
  };
}
