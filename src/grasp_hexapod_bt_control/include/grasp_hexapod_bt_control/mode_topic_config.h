// 模式话题名 YAML 加载器（config/mode_topics.yaml，yaml-cpp 直读）。
//
// 约定：文件按模式分节，节名 = switch_mode 的模式名；shared/services/bt 为
// 框架保留节。每个模式类只能读到自己的节（ModeTopics），互不干扰。
#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace grasp_hexapod_bt_control {

// 抛出此异常表示 YAML 结构/缺 key 错误（消息带 节名.key 定位，方便排查）。
class TopicConfigError : public std::runtime_error {
 public:
  explicit TopicConfigError(const std::string& message)
      : std::runtime_error(message) {}
};

// 一个模式节内的 key→topic 名只读视图（节名已绑定，报错自动定位）。
class ModeTopics {
 public:
  ModeTopics() = default;

  // 必须存在：缺失时抛 TopicConfigError（消息含 节名.key）。
  std::string get(const std::string& key) const;
  // 可选：缺失时返回 fallback。
  std::string get(const std::string& key, const std::string& fallback) const;
  bool has(const std::string& key) const { return values_.count(key) != 0; }
  bool empty() const { return values_.empty(); }
  const std::string& sectionName() const { return section_; }

 private:
  friend class ModeTopicConfig;
  ModeTopics(std::string section, std::map<std::string, std::string> values)
      : section_(std::move(section)), values_(std::move(values)) {}
  std::string section_;
  std::map<std::string, std::string> values_;
};

// 整个 mode_topics.yaml 的只读访问器。
class ModeTopicConfig {
 public:
  // 默认构造为空配置（尚未 load）；节点在构造函数体内再赋值 load 结果。
  ModeTopicConfig() = default;

  // 读文件并解析；文件不存在/不是 map 结构时抛 TopicConfigError。
  static ModeTopicConfig load(const std::string& yaml_path);

  // 取一节（模式名或 shared/services/bt）；节缺失时返回空 ModeTopics，
  // 此时 get(key, fallback) 仍可用默认值工作。
  ModeTopics section(const std::string& name) const;

  // 取列表值（如 shared.joint_feedback）；缺失或非列表时抛 TopicConfigError。
  std::vector<std::string> sectionList(const std::string& section,
                                       const std::string& key) const;

  // 已出现的节名（启动日志用）。
  std::vector<std::string> sectionNames() const;

  const std::string& path() const { return path_; }

 private:
  ModeTopicConfig(std::string path, YAML::Node root)
      : path_(std::move(path)), root_(std::move(root)) {}
  std::string path_;
  YAML::Node root_ = YAML::Node(YAML::NodeType::Undefined);
};

}  // namespace grasp_hexapod_bt_control
