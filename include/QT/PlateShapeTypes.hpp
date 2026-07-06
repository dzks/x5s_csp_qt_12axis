#pragma once

#include <QMetaType>

struct PlatformPose2D
{
    double xQ{0.0};
    double yQ{0.0};
    double phi{0.0};
};

struct SystemPoseState
{
    PlatformPose2D upper_left;
    PlatformPose2D upper_right;
    PlatformPose2D lower_left;
    PlatformPose2D lower_right;
};

enum class PlatformScope
{
    UpperOnly = 0,
    LowerOnly = 1,
    UpperAndLower = 2
};

Q_DECLARE_METATYPE(SystemPoseState)
Q_DECLARE_METATYPE(PlatformScope)