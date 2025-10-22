#pragma once

#include "object.h"
#include "utils.h"

#include <vector>
#include <memory>
#include <math.h>

// COLLISION DETECTION FOR LIGHTING/SHADOW/REFLECTION

Vector3 detectCollision(const Vector3& rayOrigin,
                        const Vector3& rayDir,
                        const Vector3 lightSource,
                        const Vector3 lightIntensity,
                        const std::vector<std::unique_ptr<Object>>& objects, int depth)
{
    // depth: shadow: -1, light/reflection: >= 0 
    float t1 = INFINITY;
    bool wasCollision = false;
    bool isLight = false;
    Vector3 pixelColor(0,0,0);
    Vector3 point;
    Vector3 ambientColor(40,80,115);
    Vector3 collision_ka;
    // detect collision with all objects
    for (auto& obj : objects){
        
        if (depth == -1){
            float length = rayDir.magnitude();
            if (obj->shadow(rayOrigin, rayDir / length, length).wasCollision == true && obj->isLight == false) {
                return Vector3(1,0,0);
            }
        }
        else {
            HitInfo hit;
            if (depth > 0){
                hit = obj->shading(rayOrigin, rayDir, lightSource, lightIntensity, objects, t1);
                if (hit.k_ref.x > 0) {
                    hit.color = hit.color * (Vector3(1,1,1) - hit.k_ref) + (hit.k_ref) * detectCollision(hit.point + hit.normal * 1e-4f, rayDir - 2 * dot(hit.normal, rayDir) * hit.normal, lightSource, lightIntensity, objects, depth - 1);
                }
            }
            else {
                hit = obj->shading(rayOrigin, rayDir, lightSource, lightIntensity, objects, t1);
            }
            if (hit.wasCollision == true) {
                if (obj->isLight) {
                    pixelColor = obj->emission;
                    isLight = true;
                }
                else {
                    pixelColor = hit.color;
                    isLight = false;
                }
                t1 = hit.t;
                wasCollision = true;
                collision_ka = hit.collision_ka;
                point = hit.point;
            }
        }
        
    }

    // shadow branch, assuming no collision
    if (depth == -1) {
        return Vector3(-1,0,0);
    }

    if (wasCollision) {
        if (!isLight){
            pixelColor = detectCollision(
                point + (lightSource - point) * 1e-4f,
                lightSource - point,
                lightSource, lightIntensity,
                objects, -1).x == -1 ? pixelColor : ambientColor * collision_ka;
        }
    }
    else {
        pixelColor = ambientColor;
    }

    return pixelColor;
}