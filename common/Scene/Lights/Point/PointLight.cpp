#include "common/Scene/Lights/Point/PointLight.h"


void PointLight::ComputeSampleRays(std::vector<Ray>& output, glm::vec3 origin, glm::vec3 normal) const
{
    origin += normal * LARGE_EPSILON;
    const glm::vec3 lightPosition = glm::vec3(GetPosition());
    const glm::vec3 rayDirection = glm::normalize(lightPosition - origin);
    const float distanceToOrigin = glm::distance(origin, lightPosition);
    output.emplace_back(origin, rayDirection, distanceToOrigin);
}

float PointLight::ComputeLightAttenuation(glm::vec3 origin) const
{
    return 1.f;
}

void PointLight::GenerateRandomPhotonRay(Ray& ray) const
{
    glm::vec3 randomDirection;
    do {
        randomDirection.x = static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f;
        randomDirection.y = static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f;
        randomDirection.z = static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f;
    } while (glm::length2(randomDirection) > 1.f);
    randomDirection = glm::normalize(randomDirection);

    ray.SetRayPosition(glm::vec3(GetPosition()));
    ray.SetRayDirection(randomDirection);
}
