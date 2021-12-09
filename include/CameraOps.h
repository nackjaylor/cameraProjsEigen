/*
Part of Camera Projections using Eigen - a set of classes for computer vision transformations.
Copyright (C) 2021  Jack Naylor

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>. 
*/


#pragma once

#include "CameraCal.h"
#include "types.h"

class CameraOps {

    
    public:

        /**
         * @brief Construct a new Camera Ops object
         * 
         */
        CameraOps();

        /**
         * @brief Construct a new Camera Ops object with intrinsics
         * 
         * @param intrinsics Intrinsics of camera
         */
        CameraOps(const CameraCal& intrinsics);

        // Euler Rotation Matrices

        /**
         * @brief Euler rotation matrix about the x axis
         * 
         * @param theta_x angle around x-axis
         * @return RotationMatrix 
         */
        RotationMatrix rx_matrix(const double& theta_x);

        /**
         * @brief Euler rotation matrix around y axis
         * 
         * @param theta_y angle around y-axis
         * @return RotationMatrix 
         */
        RotationMatrix ry_matrix(const double& theta_y);

        /**
         * @brief Euler rotation matrix about the z axis
         * 
         * @param theta_z angle around z-axis
         * @return RotationMatrix 
         */
        RotationMatrix rz_matrix(const double& theta_z);


        /**
         * @brief Euler rotation matrix in order around x, y then z axes.
         * 
         * @param theta_x angle around x-axis
         * @param theta_y angle around y-axis
         * @param theta_z angle around z-axis
         * @return RotationMatrix 
         */
        RotationMatrix rotation_xyz(const double& theta_x,const double& theta_y,const double& theta_z);

        /**
         * @brief Euler rotation matrix in order around z, y then x axes.
         * 
         * @param theta_x angle around x-axis
         * @param theta_y angle around y-axis
         * @param theta_z angle around z-axis
         * @return RotationMatrix 
         */
        RotationMatrix rotation_zyx(const double& theta_x,const double& theta_y,const double& theta_z);

        // Quaternion <-> Rotation Matrix
        /**
         * @brief Turn matrix into quaternion
         * 
         * @param rotation_matrix 3x3 rotation matrix
         * @return Quat quaternion
         */
        Quat matrix_to_quaternion(const RotationMatrix& rotation_matrix);

        /**
         * @brief Turn quaternion into rotation matrix
         * 
         * @param quaternion 1x4
         * @return RotationMatrix 3x3 rotation matrix
         */
        RotationMatrix quaternion_to_matrix(const Quat& quaternion);

        // 4x4 Translation Matrix
        /**
         * @brief Creates a 4x4 matrix for translation
         * 
         * @param point Translation in x,y,z
         * @return HomogenousMatrix translation matrix
         */
        HomogenousMatrix translation_matrix(const Point& point);

        // 4x4 Extrinsics Matrices
        /**
         * @brief Extrinsic matrix from translation and euler angles
         * 
         * @param point Translation in x,y,z
         * @param theta_x angle around x-axis
         * @param theta_y angle around y-axis
         * @param theta_z angle around z-axis
         * @return HomogenousMatrix 4x4 extrinsics
         */
        HomogenousMatrix extrinsic_matrix(const Point& point, const double& theta_x,const double& theta_y,const double& theta_z);

        /**
         * @brief Extrinsic matrix from translation and quaternion
         * 
         * @param point Translation in x,y,z
         * @param quaternion Quaternion for rotation
         * @return HomogenousMatrix 4x4 extrinsics
         */
        HomogenousMatrix extrinsic_matrix(const Point& point, const Quat& quaternion);

        /**
         * @brief Extrinsic matrix from translation and rotation matrix
         * 
         * @param point Translation in x,y,z
         * @param rotation 3x3 rotation matrix
         * @return HomogenousMatrix 4x4 extrinsics
         */
        HomogenousMatrix extrinsic_matrix(const Point& point, const RotationMatrix& rotation);

        // Translation adjustment for rotated frame
        /**
         * @brief Adjusts for translation in rotated frame of reference
         * 
         * @param point Translation in x,y,z in initial frame
         * @param rotation_matrix 3x3 rotation matrix
         * @return Point Adjusted translation
         */
        Point translation_rotated_frame(const Point& point, const RotationMatrix& rotation_matrix);

        // Look-at matrix
        /**
         * @brief Implementation of "look-at" extrinsics for a camera located at a point, looking at another point
         * 
         * @param camera Camera location (x,y,z)
         * @param subject Subject location (x,y,z)
         * @param up_vector Camera vector looking up
         * @return HomogenousMatrix 4x4 extrinsics matrix
         */
        HomogenousMatrix look_at(const Point& camera, const Point& subject = Vector3d::Zero(), const Vector3d& up_vector = Vector3d::UnitZ());

        /**
         * @brief Convert 4x4 extrinsics matrix to (x,y,z) and quaternion
         * 
         * @param extrinsics 4x4 extrinsics matrix
         * @param location 3x1 vector (x,y,z)
         * @param quaternion 1x4 quaternion
         */
        void extrinsics_to_singular(const HomogenousMatrix& extrinsics, Point& location, Quat& quaternion);

        // Coordinate Transforms - Back to Camera
        // Point world_to_camera(const Point& world_coord, const HomogenousMatrix& extrinsics);
        
        /**
         * @brief Project world point to camera plane
         * 
         * @param world_coord (x,y,z) coordinate
         * @param extrinsics  4x4 extrinsics
         * @return HPoint 4x1 homogenised point
         */
        HPoint world_to_camera(const Point& world_coord, const HomogenousMatrix& extrinsics);
        
        /**
         * @brief Project 3x1 point to pixels from camera plane
         * 
         * @param camera_coord 3x1 point on camera plane
         * @return Pixel corresponding pixel
         */
        Pixel camera_to_pixel(const Point& camera_coord);

        /**
         * @brief Project 4x1 homogenous point to pixels from camera plane
         * 
         * @param camera_coord 4x1 point on camera plane
         * @return Pixel corresponding pixel
         */
        Pixel camera_to_pixel(const HPoint& camera_coord);

        /**
         * @brief Perform full projection of world point onto camera sensor and into a pixel.
         * 
         * @param world_coord 3x1 world coordinate
         * @param extrinsics 4x4 extrinsics
         * @return Pixel corresponding image pixel
         */
        Pixel world_to_pixel(const Point& world_coord, const HomogenousMatrix& extrinsics);

        // Coordinate Transforms - Back to Camera (Arrays)
        /**
         * @brief Project 3xn array of points onto camera plane
         * 
         * @param world_coord 3xn array of world points
         * @param extrinsics 4x4 extrinsics matrix
         * @return HPointArray 4xn array of points in camera plane
         */
        HPointArray world_to_camera(const PointArray& world_coord, const HomogenousMatrix& extrinsics) const;

        /**
         * @brief Project points in camera plane onto image sensor
         * 
         * @param camera_coord  3xn array of points on camera plane
         * @return PixelArray 2xn array of pixels on image sensor 
         */
        PixelArray camera_to_pixel(const PointArray& camera_coord) const;

        /**
         * @brief Project points in camera plane onto image sensor
         * 
         * @param camera_coord  4xn array of points on camera plane
         * @return PixelArray 2xn array of pixels on image sensor 
         */
        PixelArray camera_to_pixel(const HPointArray& camera_coord) const;

        /**
         * @brief Perform full projection of points onto image sensor
         * 
         * @param world_coord 3xn array of world points
         * @param extrinsics 4x4 array of extrinsics
         * @param distort boolean flag for whether to distort pixels
         * @return PixelArray 2xn array of pixels on image sensor
         */
        PixelArray world_to_pixel(const PointArray& world_coord, const HomogenousMatrix& extrinsics, const bool& distort = false) const;

        /**
         * @brief Apply distortion to points on camera plane
         *
         * @param undistorted_points 3xn array of undistorted points on camera plane
         * @param direction integer multiplier (traditionally unity) for distortion parameters
         * @return PointArray 3xn array of distorted points on camera plane
         */
        PointArray apply_distortion(const PointArray& undistorted_points) const;

        /**
         * @brief Apply distortion to points on camera plane
         *
         * @param undistorted_points 3xn array of undistorted points on camera plane
         * @param direction integer multiplier (traditionally unity) for distortion parameters
         * @return PointArray 3xn array of distorted points on camera plane
         */
        PixelArray apply_distortion(const PixelArray& undistorted_points) const;

        /**
         * @brief Remove distortion of points on camera plane
         *
         * @param distorted_points 3xn array of distorted points
         * @return PointArray 3xn array of undistorted points
         */
        PointArray remove_distortion(const PointArray& distorted_points) const;        




        // 3x4 Camera matrix
        /**
         * @brief Returns 3x4 projection matrix of intrinsics
         * 
         * @return ProjectionMatrix 
         */
        ProjectionMatrix intrinsics_projection() const;

        // Inverse Camera Matrix
        /**
         * @brief Returns inverse of 3x3 camera matrix
         * 
         * @return Matrix3d 
         */
        Matrix3d inv_camera_matrix();

        // Coordinate Transformations - Back to World
        /**
         * @brief Projects pixel back to camera plane
         * 
         * @param pixel 2x1 pixel
         * @param z Distance of camera plane (traditionally unity)
         * @return Point 3x1 point on camera plane
         */
        Point pixel_to_camera(const Pixel& pixel, const double z = 1);

        /**
         * @brief Projects point on camera plane back to world coordinates
         * 
         * @param camera_coord 3x1 camera coordinate
         * @param extrinsics 4x4 extrinsics
         * @return Point World point
         */
        Point camera_to_world(const Point& camera_coord, const HomogenousMatrix& extrinsics);

        /**
         * @brief Projects pixel back to world coordinates
         * 
         * @param pixel 2x1 pixel
         * @param extrinsics 4x4 extrinsics
         * @param z Camera plane distance
         * @return Point World point
         */
        Point pixel_to_world(const Pixel& pixel, const HomogenousMatrix& extrinsics, const double z = 1);

        // Coordinate Transformations - Back to World (Arrays)
        /**
         * @brief Projects points back to camera plane
         * 
         * @param pixel 2xn array of pixels
         * @param z Distance of camera plane
         * @return PointArray 3xn array of points on camera plane
         */
        PointArray pixel_to_camera(const PixelArray& pixel, const double& z = 1);

        /**
         * @brief Project from camera plane to world coordinates
         * 
         * @param camera_coord 3xn array of points on camera plane
         * @param extrinsics 4x4 matrix of extrinsics
         * @return PointArray 3xn array of world points
         */
        PointArray camera_to_world(const PointArray& camera_coord, const HomogenousMatrix& extrinsics);

        /**
         * @brief Project from pixel into world coordinates
         * 
         * @param pixel 2xn array of pixels
         * @param extrinsics 4x4 extrinsics matrix
         * @param z Distance of camera plane
         * @param undistort boolean flag for whether to undistort points
         * @return PointArray 3xn array of world points
         */
        PointArray pixel_to_world(const PixelArray& pixel, const HomogenousMatrix& extrinsics, const double& z = 1, const bool& undistort = false);



        // 3x4 Projection Matrix
        /**
         * @brief Projection matrix (K*[R|T])
         * 
         * @param extrinsics 4x4 extrinsics
         * @return ProjectionMatrix 3x4 projection matrix
         */
        ProjectionMatrix projection_matrix(const HomogenousMatrix& extrinsics) const;

        // Euler <-> Quaternion
        /**
         * @brief Convert euler angles to quaternion
         * 
         * @param theta_x angle around x-axis
         * @param theta_y angle around y-axis
         * @param theta_z angle around z-axis
         * @return Quat quaternion representing rotation
         */
        Quat euler_to_quaternion(const double& theta_x,const double& theta_y,const double& theta_z);

        /**
         * @brief Convert quaternion to euler angles
         * 
         * @param quaternion rotation quaternion
         * @return Vector3d 3x1 vector of euler angles
         */
        Vector3d quaternion_to_euler(const Quat& quaternion);

        /**
         * @brief Return sign of a value
         * 
         * @param value double value
         * @return int signed int
         */
        int sign(const double& value);

        /**
         * @brief Homogenise 3x1 point
         * 
         * @param point 3x1 point
         * @return HPoint 4x1 point
         */
        HPoint homogenise_point(const Point& point) const;

        /**
         * @brief Homogenise points
         * 
         * @param points 3xn point array
         * @return HPointArray 4xn point array
         */
        HPointArray homogenise_point(const PointArray& points) const;

        /**
         * @brief Homogenise 2x1 pixel
         * 
         * @param pixel 2x1 pixel
         * @return Point 3x1 homogenised pixel
         */
        Point homogenise_pixel(const Pixel& pixel) const;

        /**
         * @brief Homogenise 2xn pixels
         * 
         * @param pixels 2xn pixel array
         * @return PointArray 3xn homogenised pixels
         */
        PointArray homogenise_pixel(const PixelArray& pixels) const;

     private:
        /**
         * @brief Intrinsics stored in camera calibration format.
         * 
         */
        CameraCal m_intrinsics;
};
