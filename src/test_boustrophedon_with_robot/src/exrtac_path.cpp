#include <iostream>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& quaternion) {
    // Convert quaternion to rotation matrix
    return quaternion.toRotationMatrix();
}

Eigen::MatrixXd createRectanglePoints(double x_obj, double y_obj, const Eigen::Quaterniond& quaternion, double width = 64, double height = 49) {
    // Convert quaternion to rotation matrix
    Eigen::Matrix3d rotationMatrix = quaternionToRotationMatrix(quaternion);

    // Calculate local coordinates of rectangle corners
    Eigen::MatrixXd localPoints(4, 3);
    localPoints << -width / 2, -height / 2, 0,
                   width / 2, -height / 2, 0,
                   width / 2, height / 2, 0,
                  -width / 2, height / 2, 0;

    // Transform to global coordinates
    Eigen::MatrixXd globalPoints = localPoints * rotationMatrix.transpose() + Eigen::RowVector3d(x_obj, y_obj, 0).replicate(4, 1);

    return globalPoints;
}

void plot3DRectangle(const Eigen::MatrixXd& globalPoints) {
    // Plotting in 3D
    plt::figure();
    
    // Connect the rectangle corners in the order they were calculated
    for (int i = 0; i < 4; ++i) {
        plt::plot3({globalPoints(i, 0), globalPoints((i + 1) % 4, 0)},
                   {globalPoints(i, 1), globalPoints((i + 1) % 4, 1)},
                   {globalPoints(i, 2), globalPoints((i + 1) % 4, 2)}, "ro-");
    }

    plt::title("3D Rectangle");
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    plt::zlabel("Z-axis");
    plt::show();
}

int main() {
    // Example usage with quaternion
    double x_obj = 50;
    double y_obj = 30;
    Eigen::Quaterniond quaternion(0.7071, 0, 0, 0.7071); // Example quaternion for 45 degrees rotation about Z-axis

    Eigen::MatrixXd rectanglePoints = createRectanglePoints(x_obj, y_obj, quaternion);
    plot3DRectangle(rectanglePoints);

    return 0;
}
