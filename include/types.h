#pragma once


#include "Eigen/Core"
#include "Eigen/Geometry"


#define _USE_MATH_DEFINES
#include <cmath>

using namespace Eigen;

using HPoint = Vector4d;
using Point = Vector3d;
using Pixel = Vector2d;
using Quat = Quaterniond;
using HomogenousMatrix = Matrix4d;
using RotationMatrix = Matrix3d;
using ProjectionMatrix = Matrix<double,3,4>;

using PixelArray = Matrix<double, 2, Dynamic>;
using PointArray = Matrix<double, 3, Dynamic>;
using HPointArray = Matrix<double, 4, Dynamic>;



