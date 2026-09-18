// 模式注册表（单头文件）：每个模式在自己的 .cpp 末尾写一行
//   REGISTER_MODE("climb", ClimbMode)
// 即被调度器在启动时自动发现。新增模式不改调度器、不改别人的文件。
//
// 注册发生在静态初始化（匿名命名空间 registrar 对象）；注册表是
// Meyer 单例，无跨编译单元初始化顺序问题。
#pragma once

#include "grasp_hexapod_bt_control/mode_base.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace grasp_hexapod_bt_control {

class ModeRegistry {
 public:
  using Factory = std::function<std::unique_ptr<ModeBase>(
      ModeContext& ctx, const ModeTopics& topics)>;

  struct Entry {
    std::string name;
    Factory factory;
    bool implemented = true;   // false = 占位（switch_mode 直接报 executor not implemented）
  };

  static ModeRegistry& instance() {
    static ModeRegistry registry;
    return registry;
  }

  // 注册模式（重名后写覆盖先写，便于测试与替换）。
  void add(const std::string& name, Factory factory, bool implemented = true) {
    entries_[name] = Entry{name, std::move(factory), implemented};
  }

  bool has(const std::string& name) const {
    return entries_.count(name) != 0;
  }

  // 未实现占位（spin_search 这类执行器未落地的模式名）。
  bool implemented(const std::string& name) const {
    const auto found = entries_.find(name);
    return found != entries_.end() && found->second.implemented;
  }

  // 创建模式实例；名字未注册返回 nullptr。
  std::unique_ptr<ModeBase> create(const std::string& name, ModeContext& ctx,
                                   const ModeTopics& topics) const {
    const auto found = entries_.find(name);
    if (found == entries_.end()) {
      return nullptr;
    }
    return found->second.factory(ctx, topics);
  }

  std::vector<std::string> names() const {
    std::vector<std::string> result;
    result.reserve(entries_.size());
    for (const auto& item : entries_) {
      result.push_back(item.first);
    }
    return result;
  }

 private:
  ModeRegistry() = default;
  std::map<std::string, Entry> entries_;
};

}  // namespace grasp_hexapod_bt_control

// ---------------------------------------------------------------------------
// 自注册宏：在模式 .cpp 末尾使用。NAME 为 switch_mode 的模式名字符串，
// CLASS 为模式类名。占位模式用 REGISTER_MODE_STUB。
// ---------------------------------------------------------------------------
#define MODE_REGISTER_MODE_IMPL(NAME, CLASS, IMPLEMENTED)                    \
  namespace {                                                                \
  struct ModeRegistrar_##CLASS {                                             \
    ModeRegistrar_##CLASS() {                                                \
      ::grasp_hexapod_bt_control::ModeRegistry::instance().add(              \
          NAME,                                                              \
          [](::grasp_hexapod_bt_control::ModeContext& ctx,                   \
             const ::grasp_hexapod_bt_control::ModeTopics& topics) {         \
            return std::make_unique<CLASS>(ctx, topics);                     \
          },                                                                 \
          IMPLEMENTED);                                                      \
    }                                                                        \
  };                                                                         \
  static ModeRegistrar_##CLASS g_mode_registrar_##CLASS;                     \
  }  // namespace

#define REGISTER_MODE(NAME, CLASS) MODE_REGISTER_MODE_IMPL(NAME, CLASS, true)
#define REGISTER_MODE_STUB(NAME, CLASS) \
  MODE_REGISTER_MODE_IMPL(NAME, CLASS, false)
