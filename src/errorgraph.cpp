#include <mrs_errorgraph_viewer/errorgraph.h>

namespace mrs_errorgraph_viewer
{

  std::vector<const Errorgraph::element_t*> Errorgraph::find_dependency_roots(const node_id_t& node_id, bool* loop_detected_out)
  {
    // first, make sure that the elements are connected as the graph
    prepare_graph();
    const auto elem = find_element_mutable(node_id);
    return DFS(elem, loop_detected_out);
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::DFS(element_t* from, bool* loop_detected_out)
  {
    std::vector<const element_t*> roots;
    std::vector<element_t*> open_elements;
    bool loop_detected = false;

    // prepare the first element into the open elements stack
    open_elements.push_back(from);

    // run the DFS
    while (!open_elements.empty())
    {
      auto cur_elem = open_elements.back();
      cur_elem->visited = true;
      open_elements.pop_back();
      const auto prevs = find_elements_waited_for(*cur_elem);

      // if this element doesn't have any nodes it's waiting for, it is a root
      if (prevs.empty())
        roots.push_back(cur_elem);

      // if there are any nodes it's waiting for, process them
      for (auto& el : prevs)
      {
        // if the element was not visited, add it to the open list
        if (!el->visited)
        {
          open_elements.push_back(el);
        }
        // if this element was already visited, we have a loop
        else
        {
          // add the element as a root although it's within a loop
          // so that the dependency doesn't get lost in the output
          roots.push_back(el);
          loop_detected = true;
        }

        // create the both-sided connection
        el->parents.push_back(cur_elem);
        cur_elem->children.push_back(el);
      }
    }

    if (loop_detected_out != nullptr)
      *loop_detected_out = loop_detected;
    return roots;
  }

  void Errorgraph::build_graph()
  {
    // first, make sure that the elements are connected as the graph
    // and all helper member variables are reset
    prepare_graph();
    auto last_unvisited = std::begin(elements_);
    while (last_unvisited != std::end(elements_))
    {
      const auto elem = last_unvisited->get();
      if (!elem->visited)
        DFS(elem);
      last_unvisited++;
    }
    graph_up_to_date_ = true;
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::find_all_roots()
  {
    std::vector<const element_t*> roots;
    for (const auto& el_ptr : elements_)
    {
      if (!el_ptr->is_waiting_for())
        roots.push_back(el_ptr.get());
    }
    return roots;
  }

  std::vector<const Errorgraph::element_t*> Errorgraph::find_all_leaves()
  {
    std::vector<const element_t*> leaves;
    for (const auto& el_ptr : elements_)
    {
      bool is_leaf = true;
      for (const auto& el_ptr2 : elements_)
      {
        if (el_ptr2->is_waiting_for(el_ptr->source_node))
        {
          is_leaf = false;
          break;
        }
      }
      if (is_leaf)
        leaves.push_back(el_ptr.get());
    }
    return leaves;
  }

  std::vector<Errorgraph::element_t*> Errorgraph::find_elements_waited_for(const element_t& element)
  {
    std::vector<element_t*> waited_for_elements;

    const auto waited_for_node_ids = element.waited_for_nodes();
    for (const auto& node_id_ptr : waited_for_node_ids)
    {
      element_t* const previous_el = find_element_mutable(*node_id_ptr);
      // if the element was found, add it to the list
      if (previous_el != nullptr)
        waited_for_elements.push_back(previous_el);
    }

    const auto waited_for_topic_names = element.waited_for_topics();
    for (const auto& topic_name_ptr : waited_for_topic_names)
    {
      element_t* const previous_el = find_element_mutable(*topic_name_ptr);
      // if the element was found, add it to the list
      if (previous_el != nullptr)
        waited_for_elements.push_back(previous_el);
    }

    return waited_for_elements;
  }

  Errorgraph::element_t* Errorgraph::find_element_mutable(const std::string& topic_name)
  {
    const auto elem_it = std::find_if(std::begin(elements_), std::end(elements_), [topic_name](const auto& el_ptr)
      {
        return el_ptr->topic_name == topic_name;
      });
    if (elem_it == std::end(elements_))
      return nullptr;
    else
      return elem_it->get();
  }

  Errorgraph::element_t* Errorgraph::find_element_mutable(const node_id_t& node_id)
  {
    const auto elem_it = std::find_if(std::begin(elements_), std::end(elements_), [node_id](const auto& el_ptr)
      {
        return el_ptr->source_node == node_id;
      });
    if (elem_it == std::end(elements_))
      return nullptr;
    else
      return elem_it->get();
  }

  const Errorgraph::element_t* Errorgraph::find_element(const std::string& topic_name)
  {
    return find_element_mutable(topic_name);
  }

  const Errorgraph::element_t* Errorgraph::find_element(const node_id_t& node_id)
  {
    return find_element_mutable(node_id);
  }

  void Errorgraph::prepare_graph()
  {
    // use this old-school iteration to not iterate
    // over new elements that may possibly be created
    // within the loop
    const int n_elements = elements_.size();
    for (int it = 0; it < n_elements; it++)
    {
      element_t* const el_ptr = elements_.at(it).get();
      el_ptr->children.clear();
      el_ptr->parents.clear();
      el_ptr->visited = false;

      // initialize all nodes this node is waiting for if they do not exist
      for (const auto& node_id_ptr : el_ptr->waited_for_nodes())
      {
        const element_t* previous_el = find_element(*node_id_ptr);
        // if the element was not found, it may have not reported yet, initialize it
        if (previous_el == nullptr)
          add_new_element(*node_id_ptr);
      }

      // initialize all nodes this node is waiting for if they do not exist
      for (const auto& topic_name_ptr : el_ptr->waited_for_topics())
      {
        const element_t* previous_el = find_element(*topic_name_ptr);
        // if the element was not found, it may have not reported yet, initialize it
        if (previous_el == nullptr)
          add_new_element(*topic_name_ptr);
      }
    }
    graph_up_to_date_ = false;
  }

  Errorgraph::element_t* Errorgraph::add_new_element(const std::string& topic_name, const node_id_t& node_id)
  {
    auto new_element_ptr = std::make_unique<element_t>(last_element_id++, topic_name, node_id);
    element_t* element = new_element_ptr.get();
    elements_.push_back(std::move(new_element_ptr));
    return element;
  }

  Errorgraph::element_t* Errorgraph::add_new_element(const node_id_t& node_id)
  {
    auto new_element_ptr = std::make_unique<element_t>(last_element_id++, node_id);
    element_t* element = new_element_ptr.get();
    elements_.push_back(std::move(new_element_ptr));
    return element;
  }

  const Errorgraph::element_t* Errorgraph::add_element_from_msg(const errorgraph_element_msg_t& msg)
  {
    const node_id_t& source_node_id = node_id_t::from_msg(msg.source_node);
    element_t* element = find_element_mutable(source_node_id);
    // if this element doesn't exist yet, construct it
    if (element == nullptr)
      element = add_new_element(source_node_id);

    element->stamp = msg.stamp;
    element->errors.clear();
    for (const auto& error_msg : msg.errors)
      element->errors.emplace_back(error_msg);

    graph_up_to_date_ = false;
    return element;
  }

  void Errorgraph::write_dot(std::ostream& os)
  {
    build_graph();

    const auto now = ros::Time::now();
    os << "digraph G\n{\n";
    for (const auto& element : elements_)
    {
      // define the element itself
      os << "  " << element->element_id << " [label=<<B><U>";

      // use either the node ID or the topic name as the element label
      // based on its type
      switch (element->type)
      {
        case element_t::type_t::topic:
          os << element->topic_name; break;
        case element_t::type_t::node:
          os << element->source_node; break;
      }
      os << "</U></B><BR/>";

      // add more info about the element
      if (element->is_not_reporting())
        os << "not reporting";
      else
        os << "age: " << (now-element->stamp).toSec() << "s";
      for (const auto& error : element->errors)
      {
        if (error.is_waiting_for() || error.is_no_error())
          continue;
        os << "<BR/>" << error.type;
      }
      os << ">";

      // draw a shape of the element based on its type
      // for topics, use a diamod shape
      if (element->type == element_t::type_t::topic)
        os << " shape=diamond";
      // if it is root, mark it with a box shape
      else if (element->children.empty())
        os << " shape=box";
      // otherwise, it's just a normal node, so leave the default shape

      // if the node has not reported yet, mark it with a red color
      if (element->is_not_reporting())
        os << " color=red";

      os << "];\n";

      // define connections to all children
      for (const auto& child : element->children)
        os << "  " << element->element_id << " -> "/*>*/ << child->element_id << "[label=\"waiting for\"];\n";
    }
    os << "}";
  }
}
