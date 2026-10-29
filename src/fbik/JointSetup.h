#pragma once
#include "SkeletonPose.h"

namespace fbik::joints
{
using namespace MeshTools;
using viewer::skeleton::Matrix;
using viewer::skeleton::Skeleton;

struct CharacterFrame { Vec3 right, forward, up; };
struct Limits
{
    bool hinge = false;
    float lower = -5, upper = 5, swing = 5; // Degrees; swing is the cone half-angle.
};
struct Setup
{
    Limits limits;
    Mat33 parentFrame, childFrame; // Character-space orientations in the rest pose.
};

std::string boneName(const std::string &path);
std::vector<Matrix> restPose(const Skeleton &sk);
CharacterFrame characterFrame(const Skeleton &sk, const std::vector<Matrix> &rest,
                              const std::vector<Vec3> &skin, const Matrix &characterWorld);
Setup setup(const Skeleton &sk, const std::vector<Matrix> &rest, const CharacterFrame &frame, size_t joint);
} // namespace fbik::joints
