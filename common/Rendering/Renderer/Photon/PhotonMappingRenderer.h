#pragma once

#include "common/Rendering/Renderer.h"
#include "common/Rendering/Renderer/Photon/Photon.h"
#include <kdtree++/kdtree.hpp>
#include <functional>
#include "common/Scene/Geometry/Mesh/MeshObject.h"
#include "common/Rendering/Renderer/Backward/BackwardRenderer.h"

class PhotonMappingRenderer : public BackwardRenderer
{
public:
    PhotonMappingRenderer(std::shared_ptr<class Scene> scene, std::shared_ptr<class ColorSampler> sampler);
    virtual void InitializeRenderer() override;
    glm::vec3 ComputeSampleColor(const struct IntersectionState& intersection, const class Ray& fromCameraRay) const override;

    void SetNumberOfDiffusePhotons(int diffuse);
    void SetNumberOfCasuticPhotons(int caustic);
    void SetPhotonGatherRange(float range);
    void SetMaxPhotonBounces(int bounces);
    void SetLightIntensityMultiplier(float mult);
    void SetFinalGatherSamples(int samples);
private:
    glm::vec3 ComputeSampleColorHelper(const struct IntersectionState& intersection, const class Ray& fromCameraRay, int finalGatherBouncesLeft, bool isInitial) const;
    glm::vec3 ComputePhotonContributionAtLocation(const struct IntersectionState& intersection, const class Ray& fromCameraRay) const;
    glm::vec3 HemisphereRandomSample() const;
    using PhotonKdtree = KDTree::KDTree<3, Photon, PhotonAccessor>;
    PhotonKdtree diffuseMap;
    PhotonKdtree causticMap;

    int diffusePhotonNumber;
    int causticPhotonNumber;
    float photonGatherRange;
    int maxPhotonBounces;
    float lightIntensityMultiplier;
    int finalGatherSamples;
    int finalGatherBounces;
    int photonsUsedPerSample;

    void GenericPhotonMapGeneration(PhotonKdtree& photonMap, int totalPhotons, std::function<bool(const class MeshObject&)> objectFilter);
    void TracePhoton(PhotonKdtree& photonMap, Ray* photonRay, glm::vec3 lightIntensity, std::vector<char>& path, float currentIOR, int remainingBounces);
};
