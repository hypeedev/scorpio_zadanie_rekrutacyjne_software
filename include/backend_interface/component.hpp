#pragma once

#include <functional>

namespace backend_interface {
  template <typename Send, typename Receive>
  class Component {
  public:
    virtual ~Component() = default;
    virtual void add_data_callback(std::function<void(const Receive&)>) = 0;
    virtual void send_data(const Send&) = 0;
  };
}
