#pragma once

namespace GraphicsLab::Geometry::BRep {

struct ParamRange {
    double start() const {
        return start_;
    }
    double end() const {
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

  private:
    double start_ = 0.0;
    double end_ = 1.0;
};

} // namespace GraphicsLab::Geometry::BRep