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

#include "Eigen/Core"


class CameraCal {

    public:

        double m_fx = 300e-3, m_fy = m_fx;
        int m_img_w = 1024, m_img_h = m_img_w;
        double m_pix_size = 5.5e-6;
        double m_k1 = 0, m_k2 = 0, m_k3 = 0;
        double m_p1 = 0, m_p2 = 0;
        double m_b1 = 0, m_b2 = 0;
        double m_ppx = m_img_w/2, m_ppy = m_img_h/2;
        Eigen::Matrix3d m_camera_matrix = new_matrix();

    public:

        /**
         * @brief Construct a new Camera Cal object
         * 
         * @param fx focal length in x (m)
         * @param img_w width/height of image in pixels
         */
        CameraCal(double fx, int img_w);

        /**
         * @brief Construct a new Camera Cal object
         * 
         * @param fx focal length in x (m)
         * @param fy focal length in y (m)
         * @param img_w image width in pixels
         * @param img_h image height in pixels
         */
        CameraCal(double fx, double fy, int img_w, int img_h);

        /**
         * @brief Construct a new Camera Cal object
         * 
         * @param fx focal length in x (m)
         * @param fy focal length in y (m)
         * @param img_w image width in pixels
         * @param img_h image height in pixels
         * @param ppx principal point in x
         * @param ppy principal point in y
         * @param k1 K1 distortion parameter
         * @param k2 K2 distortion parameter
         * @param k3 K3 distortion parameter
         * @param p1 P1 distortion parameter
         * @param p2 P2 distortion parameter
         */
        CameraCal(double fx, double fy, int img_w, int img_h, double ppx, double ppy, double k1, double k2, double k3, double p1, double p2);
        
        /**
         * @brief Construct a new Camera Cal object
         * 
         * @param fx focal length in x (m)
         * @param fy focal length in y (m)
         * @param img_w image width in pixels
         * @param img_h image height in pixels
         * @param ppx principal point in x
         * @param ppy principal point in y
         * @param k1 K1 distortion parameter
         * @param k2 K2 distortion parameter
         * @param k3 K3 distortion parameter
         * @param p1 P1 distortion parameter
         * @param p2 P2 distortion parameter
         * @param b1 B1 distortion parameter
         * @param b2 B2 distortion parameter
         */
        CameraCal(double fx, double fy, int img_w, int img_h, double ppx, double ppy, double k1, double k2, double k3, double p1, double p2,double b1, double b2);
    
        /**
         * @brief Return principal point in x
         * 
         * @return double ppx value
         */
        double getPPx();

        /**
         * @brief Get the Fy object
         * 
         * @return double fy value
         */
        double getFy();

        /**
         * @brief Construct camera matrix
         * 
         */
        void constructMatrix();

        /**
         * @brief Recreate and return matrix
         * 
         * @return Eigen::Matrix3d 3x3 camera matrix
         */
        Eigen::Matrix3d new_matrix();

        /**
         * @brief Get the Matrix object
         * 
         * @return Eigen::Matrix3d 3x3 camera matrix
         */
        Eigen::Matrix3d getMatrix() const;

        /**
         * @brief Comparison between camera cal instances
         * 
         * @param lhs 
         * @param rhs 
         * @return true if camera matrices are the same
         * @return false if camera matrices are different
         */
        friend bool operator== (const CameraCal& lhs, const CameraCal& rhs);

};
