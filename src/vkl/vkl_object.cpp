#include "vkl/vkl_object.hpp"

#include "glm/gtc/quaternion.hpp"
#include "vkl/io/assimp_loader.hpp"

VklObject::VklObject(VklDevice &device, VklObject::ImportBuilder builder) : device_(device) {
    AssimpLoader assimpLoader;
    auto modelBuilders = assimpLoader.read_model(builder.modelPath);

    for (auto modelBuilder : modelBuilders) {
        this->models.push_back(new VklModel(device, modelBuilder));
    }

    modelScaling = glm::vec3(0.2f, 0.2f, 0.2f);
    modelTranslation = glm::vec3(0, 0, 0);
    modelRotation = glm::quat(0.0f, 0.0f, 1.0f, 0.0f);
}

VklObject::~VklObject() {
    for (auto model : models) {
        delete model;
    }
}

void VklObject::render_object() {
}

void VklObject::allocDescriptorSets(VklDescriptorSetLayout &setLayout, VklDescriptorPool &pool) {
    for (auto model : models) {
        model->allocDescriptorSets(setLayout, pool);
    }
}

int VklObject::get_triangle_num() {
    int result = 0;
    for (auto model : models) {
        result += model->get_triangle_num();
    }
    return result;
}

glm::mat4 VklObject::getModelTransformation() {
    glm::mat4 model(1.0f);
    model = glm::translate(model, modelTranslation);
    model = glm::rotate(model, modelRotation.w, glm::axis(modelRotation));
    model = glm::scale(model, modelScaling);

    return model;
}
