#pragma once

#include "utils.h"

#include <vector>
#include <memory>
#include <math.h>

// OBJECT CLASSES AND STRUCTS

struct HitInfo {
    bool   wasCollision = false;
    float  t = INFINITY;
    Vector3  k_ref{0,0,0};
    Vector3 collision_ka{0,0,0};
    Vector3 color{0,0,0};
    Vector3 point{0,0,0};
    Vector3 normal{0,0,0};
};



class Object {
    public:
        virtual ~Object() = default;
        Vector3 center;
        Matrix ROT;
        bool doRotation;
        string name;
        virtual HitInfo intersect(const Vector3& rayOrigin, const Vector3& rayDir) const = 0;
        virtual HitInfo shading(const Vector3& rayOrigin, const Vector3& rayDir, const Vector3& lightSource, const Vector3& lightIntensity, const std::vector<std::unique_ptr<Object>>& objects, float t1) const = 0;
        virtual HitInfo shadow(const Vector3& rayOrigin, const Vector3& rayDir, float dist) const = 0;

        bool isLight = false;
        Vector3 emission{0,0,0};
        
        virtual void moveObject(const Vector3& displacement) {
            center = center + displacement;
        };
        void rotateObject(const Vector3& axis) {
            // ensure unit axis
            float angle = 0.5;
            ROT = ROT * RotationMatrix(axis, angle).T();
            doRotation = true;
        }

        std::pair<const Vector3,const Vector3> rotationCheck(const Vector3& rayOrigin, const Vector3& rayDir) const {

            if (doRotation){
                const Vector3 r0 = ROT * (rayOrigin - center) + center;
                const Vector3 rD = ROT * rayDir;
                return {r0,rD};
            }
            // possibly modify
            return {rayOrigin, rayDir};
        }

        virtual void scaleObject(float factor) = 0;
        virtual void resetObject() = 0;
};

class Sphere : public Object {
    public:
        Vector3 k_diff, k_spec, k_amb, k_ref;
        float radius;
        float radius_r;
        Vector3 center_r;

        Sphere(Vector3 c, Vector3 kd, Vector3 ks, Vector3 ka, Vector3 kr, float r, string n) : k_diff(kd), k_spec(ks), k_amb(ka), k_ref(kr), radius(r), radius_r(r), center_r(c) {
            center = c;
            doRotation = false;
            name = n;
        } 

        HitInfo intersect(const Vector3& rayOrigin_c, const Vector3& rayDir_c) const override {

            auto [rayOrigin, rayDir] = rotationCheck(rayOrigin_c, rayDir_c);
            HitInfo hit;
            Vector3 ce = rayOrigin - this->center;
            float sph_a = dot(rayDir, rayDir);
            float sph_b = dot(rayDir, ce);
            float sph_c = dot(ce, ce) - this->radius * this->radius;
            float D = sph_b * sph_b - sph_a * sph_c;
            if (D >= 0){
                hit.t = (-sph_b - sqrt(D)) / sph_a;
                if (hit.t < 0){
                    hit.t = (-sph_b + sqrt(D)) / sph_a;
                }
            }

            return hit;
        }

        HitInfo shading(const Vector3& rayOrigin, const Vector3& rayDir, const Vector3& lightSource, const Vector3& lightIntensity, const std::vector<std::unique_ptr<Object>>& objects, float t1) const override {
            HitInfo hit = intersect(rayOrigin, rayDir);
            if (hit.t < t1 && hit.t > 0) {
                hit.wasCollision = true;
                hit.collision_ka = this->k_amb;
                Vector3 point = (rayOrigin + hit.t * rayDir);
                hit.point = point;
                Vector3 normal = (point - this->center) / this->radius;
                hit.color = LambertShading(normal, lightSource, point, this->k_diff, lightIntensity) 
                            + PhongIllumination(normal, rayDir, lightSource, this->k_spec, lightIntensity);

                hit.k_ref = this->k_ref;
                hit.normal = normal;
            }

            return hit;
        }

        HitInfo shadow(const Vector3& rayOrigin, const Vector3& rayDir, float dist) const override {
            HitInfo hit = intersect(rayOrigin, rayDir);
            hit.wasCollision = hit.t < dist && hit.t > 1e-4f ? true : false;
            return hit;
        }
    
        void scaleObject(float factor) override {
            radius *= factor;
        }

        void resetObject() override{
            radius = radius_r;
            center = center_r;
            ROT = Matrix();
        }

};


class Plane : public Object{
    public:
        Vector3 a, b, c, k_diff, k_spec, k_amb, k_ref;
        Vector3 a_r, b_r, c_r;
        Plane(Vector3 a, Vector3 b, Vector3 c, Vector3 kd, Vector3 ks, Vector3 ka, Vector3 kr, string n) : a(a), b(b), c(c), k_diff(kd), k_spec(ks), k_amb(ka), k_ref(kr), a_r(a), b_r(b), c_r(c) {
            center = a;
            name = n;
        }

        HitInfo intersect(const Vector3& rayOrigin_c, const Vector3& rayDir_c) const override {

            auto [rayOrigin, rayDir] = rotationCheck(rayOrigin_c, rayDir_c);

            HitInfo hit;
            Matrix mat_plane(this->a - this->b, this->a - this->c, rayDir);
            Matrix mat_plane_t(this->a - this->b,
                                this->a - this->c,
                                this->a - rayOrigin);

            hit.t = mat_plane_t.det() / mat_plane.det();

            return hit;
        }

        HitInfo shading(const Vector3& rayOrigin, const Vector3& rayDir,
                        const Vector3& lightSource, const Vector3& lightIntensity,
                        const std::vector<std::unique_ptr<Object>>& objects, float t1) const override{
            HitInfo hit = intersect(rayOrigin, rayDir);
            if (hit.t < t1 && hit.t > 0) {
                hit.wasCollision = true;
                hit.collision_ka = this->k_amb;
                hit.k_ref = this->k_ref;
                Vector3 point = (rayOrigin + hit.t * rayDir);
                hit.point = point;

                Vector3 normal = cross(this->b - this->a, this->c - this->a);
                
                if (dot(normal, rayDir) > 0) normal = normal * -1;
                normal = normal / normal.magnitude();
                hit.normal = normal;

                Vector3 reflectedRayDir = rayDir - 2*(dot(rayDir, normal)) * normal;
                reflectedRayDir = reflectedRayDir / reflectedRayDir.magnitude();

                hit.color = (LambertShading(normal, lightSource, point, this->k_diff, lightIntensity)
                            + PhongIllumination(normal, rayDir, lightSource, this->k_spec, lightIntensity));
            }

            return hit;
        }

        HitInfo shadow(const Vector3& rayOrigin, const Vector3& rayDir, float dist) const override {
            HitInfo hit = intersect(rayOrigin, rayDir);
            hit.wasCollision = hit.t < dist && hit.t > 1e-4f ? true : false;
            return hit;
        }

        void moveObject(const Vector3& disp) override{
            a = a + disp;
            b = b + disp;
            c = c + disp;
        }

        void scaleObject(float factor) override {}

        void resetObject() override {
            a = a_r;
            b = b_r;
            c = c_r;
            ROT = Matrix();
        }
};


class Ellipsoid : public Object{
    public:
    Vector3 shape, k_diff, k_spec, k_amb, k_ref;
    Vector3 center_r, shape_r;
    Ellipsoid(Vector3 c, Vector3 s, Vector3 kd, Vector3 ks, Vector3 ka, Vector3 kr, string n) : shape(s), k_diff(kd), k_spec(ks), k_amb(ka), k_ref(kr), center_r(c), shape_r(s) {
        center = c;
        name = n;
    }

    HitInfo intersect(const Vector3& rayOrigin_c, const Vector3& rayDir_c) const override{

        auto [rayOrigin, rayDir] = rotationCheck(rayOrigin_c, rayDir_c);
        HitInfo hit;
        // compute coefficients a,b,c for quadratic equation
        // all elementwise
        // d^2 / (a^2, b^2, c^2) elementwise
        float el_a = ((rayDir * rayDir) / (this->shape * this->shape)).sum();
        float el_b = 2 * (rayDir * (rayOrigin - this->center) / (this->shape * this->shape)).sum();
        Vector3 intermediate_c = (rayOrigin - this->center) / (this->shape);
        float el_c = (intermediate_c * intermediate_c).sum() - 1;
        
        float D = el_b * el_b - 4 * el_a * el_c;
        if (D >= 0){
            
            hit.t = (-el_b - sqrt(D))/(2*el_a);
            if (hit.t < 0){
                hit.t = (-el_b + sqrt(D))/(2*el_a);
            }
        }

        return hit;
    }

    HitInfo shading(const Vector3& rayOrigin, const Vector3& rayDir, 
                    const Vector3& lightSource, const Vector3& lightIntensity, 
                    const std::vector<std::unique_ptr<Object>>& objects, float t1) const override {

        HitInfo hit = intersect(rayOrigin, rayDir);
        if (hit.t > 0 && hit.t < t1) {
                hit.wasCollision = true;
                hit.collision_ka = this->k_amb;
                Vector3 point = rayOrigin + hit.t * rayDir;
                // normal = gradient of implicit equation
                Vector3 normal = (point - this->center) / (this->shape * this->shape);
                normal = normal / normal.magnitude();
                hit.color = LambertShading(normal, lightSource, point, this->k_diff, lightIntensity)
                            + PhongIllumination(normal, rayDir, lightSource, this->k_spec, lightIntensity);
                
                hit.k_ref = this->k_ref;
                hit.normal = normal;

        }

        return hit;
    }

    HitInfo shadow(const Vector3& rayOrigin, const Vector3& rayDir, float dist) const override {
        HitInfo hit = intersect(rayOrigin, rayDir);
        hit.wasCollision = hit.t < dist && hit.t > 1e-4f ? true : false;
        return hit;
    }
        
    void scaleObject(float factor) override {
        shape = shape * factor;
    }

    void resetObject() override {
        center = center_r;
        shape = shape_r;

        ROT = Matrix();
    }
};