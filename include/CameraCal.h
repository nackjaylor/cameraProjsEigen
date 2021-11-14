#pragma once

#include "Eigen/Core"


class CameraCal {

    private:

        double m_fx = 300e-3, m_fy = m_fx;
        int m_img_w = 1024, m_img_h = m_img_w;
        double m_pix_size = 5.5e-6;
        double m_k1 = 0, m_k2 = 0, m_k3 = 0;
        double m_p1 = 0, m_p2 = 0;
        double m_ppx = m_img_w/2, m_ppy = m_img_h/2;
        Eigen::Matrix3d m_camera_matrix = new_matrix();

    public:


        CameraCal(double fx, int img_w);
        CameraCal(double fx, double fy, int img_w, int img_h);
        CameraCal(double fx, double fy, int img_w, int img_h, double ppx, double ppy, double k1, double k2, double k3, double p1, double p2);
        double getPPx();
        double getFy();


        void constructMatrix();

        Eigen::Matrix3d new_matrix();

        Eigen::Matrix3d getMatrix() const;
        friend bool operator== (const CameraCal& lhs, const CameraCal& rhs);

};