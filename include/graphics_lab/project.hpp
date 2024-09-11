#pragma once

/**
 * Interface for Graphics Lab Application
 */
class IGraphicsLabProject {
  public:
    virtual ~IGraphicsLabProject() = default;

    virtual void render() = 0;

    virtual void tick() = 0;
};