#pragma once

namespace GraphicsLab::Geometry::BRep {

struct ParamRange {
    [[nodiscard]] double start() const {
        return start_;
    }
    [[nodiscard]] double end() const {
        return end_;
    }

    explicit ParamRange() : start_(0), end_(1.0) {
    }

    explicit ParamRange(double start, double end) : start_(start), end_(end) {
    }

    void set_range(const double start, const double end) {
        start_ = start;
        end_ = end;
    }

    void set_start(double start) {
        start_ = start;
    }
    void set_end(double end) {
        end_ = end;
    }

    [[nodiscard]] bool is_upper_infinite() const {
        return is_upper_infinite_;
    }

    [[nodiscard]] bool is_lower_infinite() const {
        return is_lower_infinite_;
    }

    void set_upper_infinite(bool is_upper_infinite = true) {
        is_upper_infinite_ = is_upper_infinite;
    }

    void set_lower_infinite(bool is_lower_infinite = true) {
        is_lower_infinite_ = is_lower_infinite;
    }

    void set_both_infinite() {
        is_upper_infinite_ = true;
        is_lower_infinite_ = true;
    }
    [[nodiscard]] double get_mid() const {
        return start_ + (end_ - start_) / 2.0;
    }

    [[nodiscard]] bool contains(const double value) const {
        return start_ <= value && value <= end_;
    }

  private:
    double start_ = 0.0;
    double end_ = 1.0;

    bool is_upper_infinite_ = false;
    bool is_lower_infinite_ = false;
};

} // namespace GraphicsLab::Geometry::BRep