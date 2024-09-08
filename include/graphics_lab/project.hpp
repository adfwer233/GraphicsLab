#pragma once

/**
 * Interface for Graphics Lab Application
 */
class IGraphicsLabProject {
public:
    virtual ~IGraphicsLabProject() = default;

    virtual void tick();
};