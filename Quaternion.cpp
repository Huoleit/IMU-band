#include "Quaternion.hpp"

Quaternion::Quaternion(const float euler_angle[3])
{
    float cosPhi_2 = cosf(euler_angle[0] / 2.0f);
    float sinPhi_2 = sinf(euler_angle[0] / 2.0f);
    float cosTheta_2 = cosf(euler_angle[1] / 2.0f);
    float sinTheta_2 = sinf(euler_angle[1] / 2.0f);
    float cosPsi_2 = cosf(euler_angle[2] / 2.0f);
    float sinPsi_2 = sinf(euler_angle[2] / 2.0f);

    this->w =
        (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
    this->x =
        (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
    this->y =
        (cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
    this->z =
        (cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

Quaternion::Quaternion(const float rotation_matrix[3][3])
{
    float tr =
        rotation_matrix[0][0] + rotation_matrix[1][1] + rotation_matrix[2][2];
    float q[4];

    if (tr > 0.0f)
    {
        float s = sqrtf(tr + 1.0f);
        q[0] = s * 0.5f;
        s = 0.5f / s;
        q[1] = (rotation_matrix[2][1] - rotation_matrix[1][2]) * s;
        q[2] = (rotation_matrix[0][2] - rotation_matrix[2][0]) * s;
        q[3] = (rotation_matrix[1][0] - rotation_matrix[0][1]) * s;
    }
    else
    {
        /* Find maximum diagonal element in dcm
         * store index in dcm_i */
        int dcm_i = 0;

        uint8_t i;
        for (i = 1; i < 3; i++)
        {
            if (rotation_matrix[i][i] > rotation_matrix[dcm_i][dcm_i])
            {
                dcm_i = i;
            }
        }

        int dcm_j = (dcm_i + 1) % 3;
        int dcm_k = (dcm_i + 2) % 3;
        float s = sqrtf((rotation_matrix[dcm_i][dcm_i] -
                         rotation_matrix[dcm_j][dcm_j] -
                         rotation_matrix[dcm_k][dcm_k]) +
                        1.0f);
        q[dcm_i + 1] = s * 0.5f;
        s = 0.5f / s;
        q[dcm_j + 1] =
            (rotation_matrix[dcm_i][dcm_j] + rotation_matrix[dcm_j][dcm_i]) * s;
        q[dcm_k + 1] =
            (rotation_matrix[dcm_k][dcm_i] + rotation_matrix[dcm_i][dcm_k]) * s;
        q[0] =
            (rotation_matrix[dcm_k][dcm_j] - rotation_matrix[dcm_j][dcm_k]) * s;
    }

    this->w = q[0];
    this->x = q[1];
    this->y = q[2];
    this->z = q[3];
    this->normalize();
}

Quaternion::Quaternion(const float w[3], const float dt)
{
    float norm_w = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    float theta = norm_w * dt;

    if (theta > 0.03f)
    {
        this->w = cosf(theta / 2);
        this->x = (w[0] / norm_w) * sinf(theta / 2);
        this->y = (w[1] / norm_w) * sinf(theta / 2);
        this->z = (w[2] / norm_w) * sinf(theta / 2);
    }
    else
    {
        this->w = cosf(theta / 2);
        this->x = w[0] * (dt / 2);
        this->y = w[1] * (dt / 2);
        this->z = w[2] * (dt / 2);
    }
}

float Quaternion::norm()
{
    return sqrtf(this->w * this->w + this->x * this->x + this->y * this->y +
                 this->z * this->z);
}

void Quaternion::normalize(void)
{
    float norm = this->norm();
    if (norm > 0.000001f)
    {
        this->w /= norm;
        this->x /= norm;
        this->y /= norm;
        this->z /= norm;
    }
    // TODO:
    // ANCHOR  need to change norm to be invNorm
}
float Quaternion::invNorm(void)
{
    float num = this->w * this->w + this->x * this->x + this->y * this->y +
                this->z * this->z;

    float halfnum = 0.5f * num;
    float temp = num;
    long i = *(long *)&temp;
    i = 0x5f3759df - (i >> 1);
    temp = *(float *)&i;
    temp = temp * (1.5f - (halfnum * temp * temp));
    return temp;
}

void Quaternion::set(float w, float x, float y, float z)
{
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
}

void Quaternion::toEulerAngle( float *roll,  float *pitch,
                               float *yaw) const
{
    // roll (x-axis rotation)
    float sinr_cosp = +2.0f * (w * x + y * z);
    float cosr_cosp = +1.0f - 2.0f * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = +2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2.0f, sinp);  // use 90 degrees if out of range
    else
        *pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = +2.0f * (w * z + x * y);
    float cosy_cosp = +1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

Quaternion operator+(const Quaternion &lhs, const Quaternion &rhs)
{
    Quaternion sum = lhs;
    sum += rhs;
    return sum;
}

Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs)
{
    Quaternion mult = lhs;
    mult *= rhs;
    return mult;
}

Quaternion operator*(const Quaternion &lhs, const float scalar)
{
    Quaternion result = lhs;
    result *= scalar;
    return result;
}

Quaternion operator/(const Quaternion &lhs, const float scalar)
{
    Quaternion result = lhs;
    result /= scalar;
    return result;
}

Quaternion operator*(const float scalar, const Quaternion &rhs)
{
    Quaternion result = rhs;
    return result *= scalar;
}
