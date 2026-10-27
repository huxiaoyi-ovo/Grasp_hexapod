// 模式话题名 YAML 加载器实现。
#include "grasp_hexapod_bt_control/mode_topic_config.h"

#include <cstdio>

namespace grasp_hexapod_bt_control {

namespace {

std::string missingKey(const std::string& section, const std::string& key) {
  return "mode_topics.yaml[" + section + "." + key + "] is missing";
}

}  // namespace

std::string ModeTopics::get(const std::string& key) const {
  const auto found = values_.find(key);
  if (found == values_.end()) {
    throw TopicConfigError(missingKey(section_, key));
  }
  return found->second;
}

std::string ModeTopics::get(const std::string& key,
                            const std::string& fallback) const {
  const auto found = values_.find(key);
  return found == values_.end() ? fallback : found->second;
}

ModeTopicConfig ModeTopicConfig::load(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::BadFile& error) {
    throw TopicConfigError("cannot open mode_topics config: " + yaml_path +
                           " (" + error.what() + "); set ~topics_config");
  } catch (const YAML::ParserException& error) {
    throw TopicConfigError("mode_topics.yaml parse error: " +
                           std::string(error.what()));
  }
  if (!root.IsMap()) {
    throw TopicConfigError("mode_topics.yaml must be a map of sections: " +
                           yaml_path);
  }
  return ModeTopicConfig(yaml_path, root);
}

ModeTopics ModeTopicConfig::section(const std::string& name) const {
  const YAML::Node node = root_[name];
  if (!node || !node.IsMap()) {
    return ModeTopics(name, {});
  }
  std::map<std::string, std::string> values;
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    const YAML::Node value = item.second;
    // 列表值（如 shared.joint_feedback）不进标量视图，由 sectionList 读取。
    if (!value.IsScalar()) {
      continue;
    }
    values[key] = value.as<std::string>();
  }
  return ModeTopics(name, std::move(values));
}

std::vector<std::string> ModeTopicConfig::sectionList(
    const std::string& section, const std::string& key) const {
  const YAML::Node node = root_[section];
  if (!node || !node.IsMap()) {
    throw TopicConfigError(missingKey(section, key));
  }
  const YAML::Node list = node[key];
  if (!list || !list.IsSequence()) {
    throw TopicConfigError(missingKey(section, key));
  }
  std::vector<std::string> result;
  result.reserve(list.size());
  for (const auto& item : list) {
    result.push_back(item.as<std::string>());
  }
  return result;
}

std::vector<std::string> ModeTopicConfig::sectionNames() const {
  std::vector<std::string> names;
  names.reserve(root_.size());
  for (const auto& item : root_) {
    names.push_back(item.first.as<std::string>());
  }
  return names;
}

}  // namespace grasp_hexapod_bt_control
