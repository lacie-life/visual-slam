#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){

    // 3D rotation matrix directly using Matrix3d or Matrix3f
    Matrix3d rotation_matrix = Matrix3d::Identity();

    // The rotation vector uses AngleAxis, the underlying layer is not directly Matrix,
    // but the operation can be treated as a matrix (because the operator is
    // overloaded)

    //  Rotate 45 degrees along the Z axis
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));

    cout.precision(3);
    cout << "rotation matrix = \n " << rotation_vector.matrix() << endl; // convert to matrix with matrix()

    // can also be assigned directly
    rotation_matrix = rotation_vector.toRotationMatrix();

    // coordinate transformation with AngleAxis
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;

    cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;

    // Or use a rotation matrix
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

    // Euler angle: You can convert the rotation matrix directly into Euler angles
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX order, ie roll pitch yaw order

    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // Euclidean transformation matrix using Eigen::Isometry
    Isometry3d T = Isometry3d::Identity(); // Although called 3d, it is essentially a 4∗4 matrix
    T.rotate(rotation_vector); // Rotate according to rotation_vector
    T.pretranslate(Vector3d(1, 3, 4)); // Set the translation vector to (1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // Use the transformation matrix for coordinate transformation
    Vector3d v_transformed = T * v; // Equivalent to R∗v+t
    cout << "v tranformed = " << v_transformed.transpose() << endl;

    // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d.

    // Quaternion
    // You can assign AngleAxis directly to quaternions, and vice versa
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;

    // Note that the order of coeffs is (x, y, z, w), w is the real part, the first three are the imaginary part
    // can also assign a rotation matrix to it
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
    // Rotate a vector with a quaternion and use overloaded multiplication
    v_rotated = q * v; // Note that the math is qvq^{−1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // expressed by regular vector multiplication, it should be calculated as follows
    cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    return 0;
}
