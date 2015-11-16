#include "common/Rendering/Renderer/Photon/PhotonMappingRenderer.h"
#include "common/Scene/Scene.h"
#include "common/Sampling/ColorSampler.h"
#include "common/Scene/Lights/Light.h"
#include "common/Scene/Geometry/Primitives/Primitive.h"
#include "common/Scene/Geometry/Mesh/MeshObject.h"
#include "common/Rendering/Material/Material.h"
#include "common/Intersection/IntersectionState.h"
#include "common/Scene/SceneObject.h"
#include "common/Scene/Geometry/Mesh/MeshObject.h"
#include "common/Rendering/Material/Material.h"
#include "glm/gtx/component_wise.hpp"

#define VISUALIZE_PHOTON_MAPPING 0
#define DISABLE_BACKWARD_RENDERER 0

PhotonMappingRenderer::PhotonMappingRenderer(std::shared_ptr<class Scene> scene, std::shared_ptr<class ColorSampler> sampler):
    BackwardRenderer(scene, sampler), diffusePhotonNumber(200000), causticPhotonNumber(1000), 
#if VISUALIZE_PHOTON_MAPPING
    photonGatherRange(0.003f),
#else
    photonGatherRange(0.03f),
#endif
    maxPhotonBounces(1000),
    lightIntensityMultiplier(10.f),
    finalGatherSamples(4000),
    finalGatherBounces(1),
    photonsUsedPerSample(200)
{
    srand(static_cast<unsigned int>(time(NULL)));
}

void PhotonMappingRenderer::InitializeRenderer()
{
    DIAGNOSTICS_TIMER(timer, "Photon Mapping Generation");
    BackwardRenderer::InitializeRenderer();
    // Generate Photon Maps
    GenericPhotonMapGeneration(diffuseMap, diffusePhotonNumber, [](const class MeshObject& mesh) {
        const Material* mat = mesh.GetMaterial();
        if (!mat) {
            return false;
        }
        return mat->HasDiffuseReflection();
    });
    diffuseMap.optimise();

    // Todo if you have time: generate the photon caustic map
}

void PhotonMappingRenderer::GenericPhotonMapGeneration(PhotonKdtree& photonMap, int totalPhotons, std::function<bool(const class MeshObject&)> objectFilter)
{
    std::vector<const MeshObject*> relevantMeshObjects;
    Box relevantMeshBoundingBox;

    size_t totalObjects = storedScene->GetTotalObjects();
    for (size_t i = 0; i < totalObjects; ++i) {
        const SceneObject& object = storedScene->GetSceneObject(i);
        for (int j = 0; j < object.GetTotalMeshObjects(); ++j) {
            const MeshObject* meshObject = object.GetMeshObject(j);
            if (!meshObject || !objectFilter(*meshObject)) {
                continue;
            }
            
            Box meshWorldSpaceBoundingBox = meshObject->GetBoundingBox().Transform(object.GetObjectToWorldMatrix());
            relevantMeshBoundingBox.IncludeBox(meshWorldSpaceBoundingBox);
            relevantMeshObjects.push_back(meshObject);
        }
    }

    // Todo if you have time: Implement projection mapping to make your photon mapper more efficient.

    float totalLightIntensity = 0.f;
    size_t totalLights = storedScene->GetTotalLights();
    for (size_t i = 0; i < totalLights; ++i) {
        const Light* currentLight = storedScene->GetLightObject(i);
        if (!currentLight) {
            continue;
        }
        totalLightIntensity = glm::length(currentLight->GetLightColor());
    }

    // Shoot photons -- number of photons for light is proportional to the light's intensity relative to the total light intensity of the scene.
    for (size_t i = 0; i < totalLights; ++i) {
        const Light* currentLight = storedScene->GetLightObject(i);
        if (!currentLight) {
            continue;
        }

        const float proportion = glm::length(currentLight->GetLightColor()) / totalLightIntensity;
        const int totalPhotonsForLight = static_cast<const int>(proportion * totalPhotons);
        const glm::vec3 photonIntensity = currentLight->GetLightColor() / static_cast<float>(totalPhotonsForLight) * lightIntensityMultiplier;
        for (int j = 0; j < totalPhotonsForLight; ++j) {
            Ray photonRay;
            std::vector<char> path;
            path.push_back('L');
            currentLight->GenerateRandomPhotonRay(photonRay);
            TracePhoton(photonMap, &photonRay, photonIntensity, path, 1.f, maxPhotonBounces);
        }
    }
}

glm::vec3 PhotonMappingRenderer::HemisphereRandomSample() const
{
    // Perform random scattering in the hemisphere above the intersection point.
    const float u1 = static_cast<float>(rand()) / RAND_MAX;
    const float u2 = static_cast<float>(rand()) / RAND_MAX;

    const float r = std::sqrt(u1);
    const float theta = 2.f * PI * u2;

    glm::vec3 diffuseReflectionDir;
    diffuseReflectionDir.x = r * std::cos(theta);
    diffuseReflectionDir.y = r * std::sin(theta);
    diffuseReflectionDir.z = std::sqrt(std::max(0.f, 1.f - u1));
    return diffuseReflectionDir;
}

void PhotonMappingRenderer::TracePhoton(PhotonKdtree& photonMap, Ray* photonRay, glm::vec3 lightIntensity, std::vector<char>& path, float currentIOR, int remainingBounces)
{
    if (remainingBounces < 0) {
        return;
    }

    assert(photonRay);
    IntersectionState state(0, 0);
    state.currentIOR = currentIOR;

    if (storedScene->Trace(photonRay, &state)) {
        const glm::vec3 intersectionPoint = state.intersectionRay.GetRayPosition(state.intersectionT);

        // Store in photon map
        Photon newPhoton;
        newPhoton.position = intersectionPoint;
        newPhoton.intensity = lightIntensity;

        newPhoton.toLightRay = *photonRay;
        newPhoton.toLightRay.SetRayDirection(newPhoton.toLightRay.GetRayDirection() * -1.f);

        photonMap.insert(newPhoton);

        // Determine whether we should do diffuse reflection, specular reflection, transmission, or die.
        const MeshObject* hitMeshObject = state.intersectedPrimitive->GetParentMeshObject();
        assert(hitMeshObject);
        const Material* hitMaterial = hitMeshObject->GetMaterial();
        assert(hitMaterial);

        const glm::vec3 baseDiffuseReflection = hitMaterial->GetBaseDiffuseReflection();
        const float diffuseL1 = glm::dot(baseDiffuseReflection, glm::vec3(1.f));

        const glm::vec3 baseSpecularReflection = hitMaterial->GetBaseSpecularReflection();
        const float specularL1 = glm::dot(baseSpecularReflection, glm::vec3(1.f));

        const glm::vec3 baseTransmittance = hitMaterial->GetBaseTransmittance();
        const float transmittanceL1 = glm::dot(baseTransmittance, glm::vec3(1.f));

        const float probabilityForSurvival = std::min(glm::compMax(baseTransmittance + baseDiffuseReflection + baseSpecularReflection), 0.99f);
        const float totalL1 = diffuseL1 + specularL1 + transmittanceL1;

        const float diffuseProbabilityThreshold = diffuseL1 / totalL1 * probabilityForSurvival;
        const float specularProbabilityThreshold = diffuseProbabilityThreshold + specularL1 / totalL1 * probabilityForSurvival;
        const float transmissionProbabilityThreshold = specularProbabilityThreshold + transmittanceL1 / totalL1 * probabilityForSurvival;
        const float russianRoulette = static_cast<float>(rand()) / RAND_MAX;

        const glm::vec3 intersectedNormal = state.ComputeNormal();
        glm::vec3 intersectionBitangent;
        if (std::abs(glm::dot(intersectedNormal, glm::vec3(1.f, 0.f, 0.f))) < 0.8f) {
            intersectionBitangent = glm::normalize(glm::cross(intersectedNormal, glm::vec3(1.f, 0.f, 0.f)));
        } else {
            intersectionBitangent = glm::normalize(glm::cross(intersectedNormal, glm::vec3(0.f, 1.f, 0.f)));
        }

        glm::vec3 intersectionTangent = glm::normalize(glm::cross(intersectedNormal, intersectionBitangent));
        const glm::mat3 tangentToWorldMatrix(intersectionTangent, intersectionBitangent, intersectedNormal);

        Ray newPhotonRay;
        const float NdR = glm::dot(photonRay->GetRayDirection(), intersectedNormal);
        const MeshObject* intersectedMesh = state.intersectedPrimitive->GetParentMeshObject();
        assert(intersectedMesh);
        const Material* currentMaterial = intersectedMesh->GetMaterial();
        assert(currentMaterial);
        float targetIOR = currentIOR;
        
        if (russianRoulette < diffuseProbabilityThreshold) {
            path.push_back('D');

            glm::vec3 diffuseReflectionDir = glm::normalize(tangentToWorldMatrix * HemisphereRandomSample());
            newPhotonRay.SetRayPosition(intersectionPoint + LARGE_EPSILON * intersectedNormal);
            newPhotonRay.SetRayDirection(diffuseReflectionDir);
        } else if (russianRoulette < specularProbabilityThreshold) {
            path.push_back('S');
            storedScene->PerformRaySpecularReflection(newPhotonRay, *photonRay, intersectionPoint, NdR, state);
        } else if (russianRoulette < transmissionProbabilityThreshold) {
            path.push_back('S');
            targetIOR = (NdR < SMALL_EPSILON) ? currentMaterial->GetIOR() : 1.f;
            storedScene->PerformRayRefraction(newPhotonRay, *photonRay, intersectionPoint, NdR, state, targetIOR);
        } else {
            return;
        }
        TracePhoton(photonMap, &newPhotonRay, lightIntensity, path, targetIOR, remainingBounces - 1);
    }
}

glm::vec3 PhotonMappingRenderer::ComputePhotonContributionAtLocation(const struct IntersectionState& intersection, const class Ray& fromCameraRay) const
{
    const MeshObject* parentObject = intersection.intersectedPrimitive->GetParentMeshObject();
    assert(parentObject);

    const Material* objectMaterial = parentObject->GetMaterial();
    assert(objectMaterial);

    Photon intersectionVirtualPhoton;
    intersectionVirtualPhoton.position = intersection.intersectionRay.GetRayPosition(intersection.intersectionT);

    glm::vec3 photonMappingColor;
    std::vector<Photon> foundPhotons;
    float gatherRangeToUse = photonGatherRange;
    diffuseMap.find_within_range(intersectionVirtualPhoton, gatherRangeToUse, std::back_inserter(foundPhotons));

#if VISUALIZE_PHOTON_MAPPING
    for (int i = 0; i < foundPhotons.size(); ++i) {
        photonMappingColor =  glm::vec3(1.f, 0.f, 0.f);
#else
    for (int i = 0; i < foundPhotons.size(); ++i) {
        photonMappingColor += objectMaterial->ComputeBRDF(intersection, foundPhotons[i].intensity, foundPhotons[i].toLightRay, fromCameraRay, 1.f, true, false);
#endif
    }
#if !VISUALIZE_PHOTON_MAPPING
    photonMappingColor /= (PI * gatherRangeToUse * gatherRangeToUse);
#endif
    return photonMappingColor;
}

glm::vec3 PhotonMappingRenderer::ComputeSampleColorHelper(const struct IntersectionState& intersection, const class Ray& fromCameraRay, int finalGatherBouncesLeft, bool isInitial) const
{
    if (!intersection.hasIntersection) {
        return glm::vec3();
    }

    const glm::vec3 intersectionPoint = intersection.intersectionRay.GetRayPosition(intersection.intersectionT);
    glm::vec3 photonMappingColor;

#if !VISUALIZE_PHOTON_MAPPING
    if (!isInitial || !finalGatherBouncesLeft) {
#endif
        photonMappingColor = ComputePhotonContributionAtLocation(intersection, fromCameraRay);
#if !VISUALIZE_PHOTON_MAPPING
    }
#endif

#if !VISUALIZE_PHOTON_MAPPING
    const MeshObject* parentObject = intersection.intersectedPrimitive->GetParentMeshObject();
    assert(parentObject);

    const Material* objectMaterial = parentObject->GetMaterial();
    assert(objectMaterial);

    if (finalGatherBouncesLeft) {
        const glm::vec3 intersectedNormal = intersection.ComputeNormal();
        glm::vec3 intersectionBitangent;
        if (std::abs(glm::dot(intersectedNormal, glm::vec3(1.f, 0.f, 0.f))) < 0.8f) {
            intersectionBitangent = glm::normalize(glm::cross(intersectedNormal, glm::vec3(1.f, 0.f, 0.f)));
        } else {
            intersectionBitangent = glm::normalize(glm::cross(intersectedNormal, glm::vec3(0.f, 1.f, 0.f)));
        }

        glm::vec3 intersectionTangent = glm::normalize(glm::cross(intersectedNormal, intersectionBitangent));
        const glm::mat3 tangentToWorldMatrix(intersectionTangent, intersectionBitangent, intersectedNormal);
        glm::vec3 finalGatherColor;

        for (int i = 0; i < finalGatherSamples; ++i) {
            IntersectionState finalGatherState;
            glm::vec3 finalGatherDir = glm::normalize(tangentToWorldMatrix * HemisphereRandomSample());
            Ray finalGatherRay(intersectionPoint + intersectedNormal * LARGE_EPSILON, finalGatherDir);
            if (storedScene->Trace(&finalGatherRay, &finalGatherState)) {
                glm::vec3 incomingRadiance = ComputeSampleColorHelper(finalGatherState, finalGatherRay, finalGatherBouncesLeft - 1, false);
                finalGatherColor += objectMaterial->ComputeBRDF(intersection, incomingRadiance, finalGatherRay, fromCameraRay, 1.f, true, false);
            }
        }
        photonMappingColor += finalGatherColor / static_cast<float>(finalGatherSamples);
    }
#endif
    return photonMappingColor;
}

glm::vec3 PhotonMappingRenderer::ComputeSampleColor(const struct IntersectionState& intersection, const class Ray& fromCameraRay) const
{
    glm::vec3 backwardRenderColor;
#if !DISABLE_BACKWARD_RENDERER
    backwardRenderColor = BackwardRenderer::ComputeSampleColor(intersection, fromCameraRay);
#endif
    return backwardRenderColor + ComputeSampleColorHelper(intersection, fromCameraRay, finalGatherBounces, true);
}

void PhotonMappingRenderer::SetFinalGatherSamples(int samples)
{
    finalGatherSamples = samples;
}

void PhotonMappingRenderer::SetNumberOfDiffusePhotons(int diffuse)
{
    diffusePhotonNumber = diffuse;
}

void PhotonMappingRenderer::SetNumberOfCasuticPhotons(int caustic)
{
    causticPhotonNumber = caustic;
}

void PhotonMappingRenderer::SetPhotonGatherRange(float range)
{
    photonGatherRange = range;
}

void PhotonMappingRenderer::SetMaxPhotonBounces(int bounces)
{
    maxPhotonBounces = bounces;
}

void PhotonMappingRenderer::SetLightIntensityMultiplier(float mult)
{
    lightIntensityMultiplier = mult;
}
