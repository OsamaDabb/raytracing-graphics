#pragma once

#include <math.h>
// VECTOR CLASS AND FUNCTIONS

class Vector3{
    public:
        float x, y, z;
        Vector3(float xi=0, float yi=0, float zi=0) : x(xi), y(yi), z(zi) {}
        
        Vector3 operator+(const Vector3& b) const {
            return Vector3(x + b.x, y + b.y, z + b.z);
        }

        Vector3 operator-(const Vector3& b) const {
            return Vector3(x - b.x, y - b.y, z - b.z);
        }

        Vector3 operator*(float c) const {
            return Vector3(c*x, c*y, c*z);
        }

        Vector3 operator*(const Vector3& b) const {
            return Vector3(x * b.x, y * b.y, z * b.z);
        }

        Vector3 operator/(float c) const{
            return Vector3(x/c,y/c,z/c);
        }

        Vector3 operator/(const Vector3& b) const{
            return Vector3(x/b.x, y/b.y, z/b.z);
        }

        float sum(){
            return x + y + z;
        }

        float magnitude() const{
            return sqrt(x*x + y*y + z*z);
        }
};

inline float dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// def from lec 2 slide 9
inline Vector3 cross(const Vector3& a, const Vector3& b){
    return Vector3(a.y*b.z - a.z*b.y,
                  a.z*b.x - a.x * b.z,
                  a.x*b.y - a.y* b.x);
}

inline void printVec(const Vector3& a){
    printf("(%f, %f, %f)\n", a.x, a.y, a.z);
}

inline Vector3 operator*(float s, const Vector3& v) {
    return v * s; // reuse your member operator
}

// HELPER CLASSES AND FUNCTIONS

class Matrix {
    public:
        Vector3 c1, c2, c3;
        Matrix() : c1(1,0,0), c2(0,1,0), c3(0,0,1) {}
        Matrix(Vector3 c1, Vector3 c2, Vector3 c3) : c1(c1), c2(c2), c3(c3) {}

        float det(){
            return c1.x * (c2.y * c3.z - c3.y * c2.z) 
                    - c2.x * (c1.y * c3.z - c3.y * c1.z) 
                    + c3.z * (c1.y * c2.z - c2.y * c1.z);
        }

        Vector3 operator*(const Vector3& v) const {
            return c1 * v.x + c2 * v.y + c3 * v.z;
        }
        Matrix operator*(const Matrix& B) const {
            // Each column of the result is A * (column of B)
            Vector3 new_c1 = (*this) * B.c1;
            Vector3 new_c2 = (*this) * B.c2;
            Vector3 new_c3 = (*this) * B.c3;

            return Matrix(new_c1, new_c2, new_c3);
        }

        Matrix T() const {
        // convert columns to rows
        Vector3 row1(c1.x, c2.x, c3.x);
        Vector3 row2(c1.y, c2.y, c3.y);
        Vector3 row3(c1.z, c2.z, c3.z);

        // your Matrix constructor expects columns, not rows
        return Matrix(row1, row2, row3);
    }


};

inline Matrix RotationMatrix(const Vector3& axis, float angle){
    Vector3 w = axis / axis.magnitude();
    Vector3 t = w.y == 0 ? Vector3(0,1,0) : Vector3(1,0,0);

    Vector3 u = cross(t, w);
    u = u / u.magnitude();
    Vector3 v = cross(w, u);

    float c = cos(angle);
    float s = sin(angle);

    Matrix orthog_trans(u,v,w);

    Matrix axis_rot(Vector3(c, s, 0), Vector3(-s, c, 0), Vector3(0,0,1));

    return orthog_trans * axis_rot * orthog_trans.T();



    // rows of the 3×3 rotation matrix
    // Vector3 row1(t*x*x + c,     t*x*y - s*z,   t*x*z + s*y);
    // Vector3 row2(t*x*y + s*z,   t*y*y + c,     t*y*z - s*x);
    // Vector3 row3(t*x*z - s*y,   t*y*z + s*x,   t*z*z + c);

    // turns them into the columns of a matrix class (transpose)
    // return Matrix(row1, row2, row3).T();
}

inline Vector3 LambertShading(const Vector3& n, const Vector3& l_s, const Vector3& p, const Vector3& kd, const Vector3& i){
    
    Vector3 l = (l_s - p);
    l = l / l.magnitude();
    if (dot(n, l) > 0) {
        return kd * i * dot(n, l);
    }
    else {
        return Vector3(0,0,0);
    }

}

inline Vector3 PhongIllumination(const Vector3& n, const Vector3& d, const Vector3& l_s, const Vector3& kd, const Vector3& I){

    const int p = 32;
    Vector3 l = l_s - d;
    l = l / l.magnitude();
    Vector3 v = (-1 * d) / d.magnitude();
    Vector3 h = (v+l);
    h = h / h.magnitude();

    if (dot(n, h) > 0){
        return kd * I * pow(dot(n, h), p);
    }
    return Vector3(0,0,0);

}