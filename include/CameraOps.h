#pragma once

#include "CameraCal.h"
#include "types.h"

class CameraOps {

    
    public:
        CameraOps();
        CameraOps(const CameraCal& intrinsics);

        // Euler Rotation Matrices
        RotationMatrix rx_matrix(const double& theta_x);
        RotationMatrix ry_matrix(const double& theta_y);
        RotationMatrix rz_matrix(const double& theta_z);

        RotationMatrix rotation_xyz(const double& theta_x,const double& theta_y,const double& theta_z);
        RotationMatrix rotation_zyx(const double& theta_x,const double& theta_y,const double& theta_z);

        // Quaternion <-> Rotation Matrix
        Quat matrix_to_quaternion(const RotationMatrix& rotation_matrix);
        RotationMatrix quaternion_to_matrix(const Quat& quaternion);

        // 4x4 Translation Matrix
        HomogenousMatrix translation_matrix(const Point& point);

        // 4x4 Extrinsics Matrices
        HomogenousMatrix extrinsic_matrix(const Point& point, const double& theta_x,const double& theta_y,const double& theta_z);
        HomogenousMatrix extrinsic_matrix(const Point& point, const Quat& quaternion);
        HomogenousMatrix extrinsic_matrix(const Point& point, const RotationMatrix& rotation);

        // Translation adjustment for rotated frame
        Point translation_rotated_frame(const Point& point, const RotationMatrix& rotation_matrix);

        // Look-at matrix
        HomogenousMatrix look_at(const Point& camera, const Point& subject = Vector3d::Zero(), const Vector3d& up_vector = Vector3d::UnitZ());

        void extrinsics_to_singular(const HomogenousMatrix& extrinsics, Point& location, Quat& quaternion);

        // Coordinate Transforms - Back to Camera
        // Point world_to_camera(const Point& world_coord, const HomogenousMatrix& extrinsics);
        HPoint world_to_camera(const Point& world_coord, const HomogenousMatrix& extrinsics);
        Pixel camera_to_pixel(const Point& camera_coord);
        Pixel camera_to_pixel(const HPoint& camera_coord);
        Pixel world_to_pixel(const Point& world_coord, const HomogenousMatrix& extrinsics);

        // Coordinate Transforms - Back to Camera (Arrays)
        // PointArray world_to_camera(const PointArray& world_coord, const HomogenousMatrix& extrinsics);
        HPointArray world_to_camera(const PointArray& world_coord, const HomogenousMatrix& extrinsics) const;
        PixelArray camera_to_pixel(const PointArray& camera_coord) const;
        PixelArray camera_to_pixel(const HPointArray& camera_coord) const;
        PixelArray world_to_pixel(const PointArray& world_coord, const HomogenousMatrix& extrinsics) const;


        




        // 3x4 Camera matrix
        ProjectionMatrix intrinsics_projection() const;

        // Inverse Camera Matrix
        Matrix3d inv_camera_matrix();

        // Coordinate Transformations - Back to World
        Point pixel_to_camera(const Pixel& pixel, const double z = 1);
        Point camera_to_world(const Point& camera_coord, const HomogenousMatrix& extrinsics);
        Point pixel_to_world(const Pixel& pixel, const HomogenousMatrix& extrinsics, const double z = 1);

        // Coordinate Transformations - Back to World (Arrays)
        PointArray pixel_to_camera(const PixelArray& pixel, const double z = 1);
        PointArray camera_to_world(const PointArray& camera_coord, const HomogenousMatrix& extrinsics);
        PointArray pixel_to_world(const PixelArray& pixel, const HomogenousMatrix& extrinsics, const double z = 1);

        // Quaternion <-> Rotation Matrix
        // Quat matrix_to_quaternion(const RotationMatrix& rotation_matrix);
        // RotationMatrix quaternion_to_matrix(const Quat& quaternion);

        // 3x4 Projection Matrix
        ProjectionMatrix projection_matrix(const HomogenousMatrix& extrinsics) const;

        // Euler <-> Quaternion
        Quat euler_to_quaternion(const double& theta_x,const double& theta_y,const double& theta_z);
        Vector3d quaternion_to_euler(const Quat& quaternion);

        int sign(const double& value);
        HPoint homogenise_point(const Point& point) const;
        HPointArray homogenise_point(const PointArray& points) const;
        Point homogenise_pixel(const Pixel& pixel) const;
        PointArray homogenise_pixel(const PixelArray& pixels) const;

     private:
        CameraCal m_intrinsics;
};




        