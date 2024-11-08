#pragma once

#include <any>
#include <map>
#include <string>

namespace GraphicsLab {
struct AnnotatedClass {
  public:
    template <typename T> AnnotatedClass& set_annotation(const std::string &name, T data) {
        annotations_[name] = data;
        return *this;
    }

    template <typename T> T get_annotation(const std::string &name) {
        return any_cast<T>(&annotations_[name]);
    }

    bool has_annotation(const std::string &name) const {
        return annotations_.contains(name);
    }

    void copy_annotation(const AnnotatedClass &source) {
        for (const auto &[name, data] : source.annotations_) {
            annotations_[name] = data;
        }
    }

  private:
    std::map<std::string, std::any> annotations_;
};
} // namespace GraphicsLab