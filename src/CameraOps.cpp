#include "CameraOps.h"
#include "CameraCal.h"
#include "Eigen/Core"

#include <cmath>

using namespace Eigen;

CameraOps::CameraOps(const CameraCal& intrinsics)
    : m_intrinsics(intrinsics) {}


RotationMatrix CameraOps::rx_matrix(const double& theta_x) {
    return AngleAxisd(theta_x,Vector3d::UnitX()).toRotationMatrix();

}

RotationMatrix CameraOps::ry_matrix(const double& theta_y) {
    return AngleAxisd(theta_y,Vector3d::UnitY()).toRotationMatrix();
    
}

RotationMatrix CameraOps::rz_matrix(const double& theta_z) {
    return AngleAxisd(theta_z,Vector3d::UnitZ()).toRotationMatrix();

}

RotationMatrix CameraOps::rotation_xyz(const double& theta_x,const double& theta_y,const double& theta_z) {
    RotationMatrix rx = rx_matrix(theta_x);
    RotationMatrix ry = ry_matrix(theta_y);
    RotationMatrix rz = rz_matrix(theta_z);
    return rx*ry*rz;

}

RotationMatrix CameraOps::rotation_zyx(const double& theta_x,const double& theta_y,const double& theta_z) {
    RotationMatrix rx = rx_matrix(theta_x);
    RotationMatrix ry = ry_matrix(theta_y);
    RotationMatrix rz = rz_matrix(theta_z);
    return rz*ry*rx;
}

Quat CameraOps::matrix_to_quaternion(const RotationMatrix& rotation_matrix) {
    Quat quaternion(rotation_matrix);
    
    return quaternion;
}

RotationMatrix CameraOps::quaternion_to_matrix(const Quat& quaternion) {
    RotationMatrix matrix = quaternion.normalized().toRotationMatrix();
    return matrix;
}

HomogenousMatrix CameraOps::translation_matrix(const Point& point) {
    HomogenousMatrix translation = HomogenousMatrix::Identity();
    translation(seq(0,last-1),last) = point;
    return translation;
}

HomogenousMatrix CameraOps::extrinsic_matrix(const Point& point, const double& theta_x,const double& theta_y,const double& theta_z) {
    RotationMatrix rotation = rotation_xyz(theta_x, theta_y, theta_z);
    Point new_point = translation_rotated_frame(point, rotation);
    HomogenousMatrix extrinsics = translation_matrix(new_point);
    extrinsics.block(0,0,3,3) = rotation;
    return extrinsics;
}

HomogenousMatrix CameraOps::extrinsic_matrix(const Point& point, const Quat& quaternion) {
    RotationMatrix rotation = quaternion_to_matrix(quaternion);
    Point new_point = translation_rotated_frame(point, rotation);
    HomogenousMatrix extrinsics = translation_matrix(new_point);
    

    extrinsics.block(0,0,3,3) = rotation;
    return extrinsics;
}

HomogenousMatrix CameraOps::extrinsic_matrix(const Point& point, const RotationMatrix& rotation) {
    Point new_point = translation_rotated_frame(point, rotation);
    HomogenousMatrix extrinsics = translation_matrix(new_point);

    extrinsics.block(0,0,3,3) = rotation;
    return extrinsics;
}

void CameraOps::extrinsics_to_singular(const HomogenousMatrix& extrinsics, Point& location, Quat& quaternion) {
    location << extrinsics(seq(0,last-1),last);
    quaternion = matrix_to_quaternion(extrinsics(seq(0,last-1),seq(0,last-1)));
}

Point CameraOps::translation_rotated_frame(const Point& point, const RotationMatrix& rotation_matrix) {
    Vector3d xaxis = rotation_matrix.row(0);
    Vector3d yaxis = rotation_matrix.row(1);
    Vector3d zaxis = rotation_matrix.row(2);

    Point out = Point::Zero();
    out(0) = -xaxis.dot(point);
    out(1) = -yaxis.dot(point);
    out(2) = -zaxis.dot(point);
    return out;
}

HomogenousMatrix CameraOps::look_at(const Point& camera, const Point& subject, const Vector3d& up_vector) {
    Vector3d zaxis = subject-camera;
    zaxis = zaxis.normalized();
    Vector3d xaxis = zaxis.cross(up_vector).normalized();
    Vector3d yaxis = xaxis.cross(zaxis);
    zaxis = -zaxis;

    HomogenousMatrix outMatrix = HomogenousMatrix::Identity();
    outMatrix(0,seq(0,last-1)) = xaxis;
    outMatrix(1,seq(0,last-1)) = yaxis;
    outMatrix(2,seq(0,last-1)) = zaxis;
    outMatrix(0,3) = -xaxis.dot(camera);
    outMatrix(1,3) = -yaxis.dot(camera);
    outMatrix(2,3) = -zaxis.dot(camera);

    return outMatrix;

}



HPoint CameraOps::world_to_camera(const Point& world_coord, const HomogenousMatrix& extrinsics) {
    HPoint homogenised = homogenise_point(world_coord);
    return extrinsics*homogenised;
}

Pixel CameraOps::camera_to_pixel(const Point& camera_coord) {
    HPoint homogenised = homogenise_point(camera_coord);
    ProjectionMatrix projection_matrix = intrinsics_projection();
    Point pixel = projection_matrix*homogenised;
    return pixel.colwise().hnormalized();
}
Pixel CameraOps::camera_to_pixel(const HPoint& camera_coord) {
    ProjectionMatrix projection_matrix = intrinsics_projection();
    Point pixel = projection_matrix*camera_coord;
    return pixel.colwise().hnormalized();

}



Pixel CameraOps::world_to_pixel(const Point& world_coord, const HomogenousMatrix& extrinsics) {
    
    HPoint homogenised = world_to_camera(world_coord,extrinsics);

    Pixel pixel = camera_to_pixel(homogenised);
    return pixel;
}




HPointArray CameraOps::world_to_camera(const PointArray& world_coord, const HomogenousMatrix& extrinsics) const {
    
    HPointArray homogenised = homogenise_point(world_coord);

    return extrinsics*homogenised;
}



PixelArray CameraOps::camera_to_pixel(const PointArray& camera_coord) const {
    HPointArray homogenised = homogenise_point(camera_coord);
    ProjectionMatrix full_intrinsics = intrinsics_projection();
    PointArray pixel = full_intrinsics*homogenised;
    return pixel.colwise().hnormalized();
}

PixelArray CameraOps::camera_to_pixel(const HPointArray& camera_coord) const {
    
    ProjectionMatrix full_intrinsics = intrinsics_projection();

    PointArray pixels = full_intrinsics*camera_coord;

    return pixels.colwise().hnormalized();

}



PixelArray CameraOps::world_to_pixel(const PointArray& world_coord, const HomogenousMatrix& extrinsics) const {
    
    HPointArray homogenised = world_to_camera(world_coord,extrinsics);
    
    PixelArray pixel = camera_to_pixel(homogenised);
    return pixel;
}






Point CameraOps::pixel_to_camera(const Pixel& pixel, const double z) {
    Point homogenised = homogenise_pixel(pixel);
    Matrix3d inv_K = inv_camera_matrix();
    return -z*(inv_K*homogenised);
}

Point CameraOps::camera_to_world(const Point& camera_coord, const HomogenousMatrix& extrinsics) {
    HPoint homogenised = homogenise_point(camera_coord);
    HPoint homog_world = extrinsics.inverse()*homogenised;
    return homog_world.colwise().hnormalized();
}

Point CameraOps::pixel_to_world(const Pixel& pixel, const HomogenousMatrix& extrinsics, const double z) {
    Point camera_coord = pixel_to_camera(pixel,z);
    Point world_coord = camera_to_world(camera_coord,extrinsics);
    return world_coord;
}


PointArray CameraOps::pixel_to_camera(const PixelArray& pixel, const double z) {
    PointArray homogenised = homogenise_pixel(pixel);
    Matrix3d inv_K = inv_camera_matrix();
    return -z*(inv_K*homogenised);
}

PointArray CameraOps::camera_to_world(const PointArray& camera_coord, const HomogenousMatrix& extrinsics) {
    HPointArray homogenised = homogenise_point(camera_coord);
    HPointArray homog_world = extrinsics.inverse()*homogenised;
    return homog_world.colwise().hnormalized();
}

PointArray CameraOps::pixel_to_world(const PixelArray& pixel, const HomogenousMatrix& extrinsics, const double z) {
    PointArray camera_coord = pixel_to_camera(pixel,z);
    PointArray world_coord = camera_to_world(camera_coord,extrinsics);
    return world_coord;
}

ProjectionMatrix CameraOps::projection_matrix(const HomogenousMatrix& extrinsics) const {
    ProjectionMatrix intrinsic = intrinsics_projection();
    return intrinsic*extrinsics;
}

Matrix3d CameraOps::inv_camera_matrix() {
    return m_intrinsics.getMatrix().inverse();
}


HPoint CameraOps::homogenise_point(const Point& point) const {
    HPoint homogenised = HPoint::Ones();
    homogenised(seq(0,last-1)) = point;

    return homogenised;
}

HPointArray CameraOps::homogenise_point(const PointArray& points) const {
    
    HPointArray homogenised;
    
    homogenised.resize(NoChange,points.cols());
    
    homogenised.setOnes();
    
    homogenised(seq(0,last-1),all) = points;

    return homogenised;
}

Point CameraOps::homogenise_pixel(const Pixel& pixel) const {
    Point homogenised = Point::Ones();
    homogenised(seq(0,last-1)) = pixel;
    return homogenised;
}

PointArray CameraOps::homogenise_pixel(const PixelArray& pixels) const {
    PointArray homogenised;
    homogenised.resize(NoChange,pixels.cols());
    homogenised.setOnes();
    homogenised(seq(0,last-1),all) = pixels;
    return homogenised;
}

int CameraOps::sign(const double& value) {
    return copysign(1,value);
}

ProjectionMatrix CameraOps::intrinsics_projection() const {
    ProjectionMatrix intrinsics_matrix;
    intrinsics_matrix.setZero();
    intrinsics_matrix(all,seq(0,last-1)) = m_intrinsics.getMatrix();
    return intrinsics_matrix;
}