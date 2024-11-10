#include "vkl/core/vkl_descriptor.hpp"
#include "spdlog/spdlog.h"
#include <stdexcept>

VklDescriptorSetLayout::Builder &VklDescriptorSetLayout::Builder::addBinding(uint32_t binding,
                                                                             VkDescriptorType descriptorType,
                                                                             VkShaderStageFlags stageFlags,
                                                                             uint32_t count) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = binding;
    layoutBinding.descriptorType = descriptorType;
    layoutBinding.stageFlags = stageFlags;
    layoutBinding.descriptorCount = count;
    bindings[binding] = layoutBinding;

    return *this;
}

std::unique_ptr<VklDescriptorSetLayout> VklDescriptorSetLayout::Builder::build() {
    return std::make_unique<VklDescriptorSetLayout>(device_, *this);
}

VklDescriptorSetLayout::Builder &VklDescriptorSetLayout::Builder::addBindingsFromResource(
    spirv_cross::Compiler &compiler, spirv_cross::ShaderResources &resources) {
    for (const auto &buffer : resources.uniform_buffers) {
        uint32_t setShaderId = compiler.get_decoration(buffer.id, spv::DecorationDescriptorSet);
        uint32_t bindingIndex = compiler.get_decoration(buffer.id, spv::DecorationBinding);

        auto &type = compiler.get_type(buffer.type_id);
        auto &type2 = compiler.get_type(buffer.base_type_id);
        auto declaredSize = static_cast<uint32_t>(compiler.get_declared_struct_size(type));

        addBinding(bindingIndex, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS);
        descriptorSetLayoutKey.uniformDescriptors.push_back(
            {.binding = bindingIndex, .size = declaredSize, .typeName = buffer.name});
    }

    for (const auto &image : resources.sampled_images) {
        uint32_t setShaderId = compiler.get_decoration(image.id, spv::DecorationDescriptorSet);
        uint32_t bindingIndex = compiler.get_decoration(image.id, spv::DecorationBinding);

        addBinding(bindingIndex, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT);
        descriptorSetLayoutKey.sampledImageBufferDescriptors.push_back({.binding = bindingIndex, .name = image.name});
    }

    return *this;
}

VklDescriptorSetLayout::~VklDescriptorSetLayout() {
    vkDestroyDescriptorSetLayout(device_.device(), descriptorSetLayout_, nullptr);
}

VklDescriptorSetLayout::VklDescriptorSetLayout(VklDevice &device, Builder &builder)
    : device_(device), bindings_(builder.bindings) {
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings;
    for (auto [key, value] : builder.bindings) {
        setLayoutBindings.push_back(value);
    }

    descriptorSetLayoutKey = std::move(builder.descriptorSetLayoutKey);

    VkDescriptorSetLayoutCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    createInfo.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
    createInfo.pBindings = setLayoutBindings.data();

    if (vkCreateDescriptorSetLayout(device_.device(), &createInfo, nullptr, &descriptorSetLayout_) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor set layout \n");
    }
}

VklDescriptorPool::Builder &VklDescriptorPool::Builder::addPoolSize(VkDescriptorType descriptorType, uint32_t count) {
    poolSizes.push_back({descriptorType, count});
    return *this;
}

VklDescriptorPool::Builder &VklDescriptorPool::Builder::setPoolFlags(VkDescriptorPoolCreateFlags flags) {
    poolFlags = flags;
    return *this;
}

VklDescriptorPool::Builder &VklDescriptorPool::Builder::setMaxSets(uint32_t count) {
    maxSets = count;
    return *this;
}

std::unique_ptr<VklDescriptorPool> VklDescriptorPool::Builder::build() const {
    return std::make_unique<VklDescriptorPool>(device_, maxSets, poolFlags, poolSizes);
}

VklDescriptorPool::VklDescriptorPool(VklDevice &device, uint32_t maxSets, VkDescriptorPoolCreateFlags poolFlags,
                                     const std::vector<VkDescriptorPoolSize> &poolSizes)
    : device_(device) {
    VkDescriptorPoolCreateInfo descriptorPoolInfo{};
    descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    descriptorPoolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    descriptorPoolInfo.pPoolSizes = poolSizes.data();
    descriptorPoolInfo.maxSets = maxSets;
    descriptorPoolInfo.flags = poolFlags;

    if (vkCreateDescriptorPool(device_.device(), &descriptorPoolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}

VklDescriptorPool::~VklDescriptorPool() {
    vkDestroyDescriptorPool(device_.device(), descriptorPool, nullptr);
}

bool VklDescriptorPool::allocateDescriptor(const VkDescriptorSetLayout descriptorSetLayout,
                                           VkDescriptorSet &descriptor) const {
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.pSetLayouts = &descriptorSetLayout;
    allocInfo.descriptorSetCount = 1;

    if (vkAllocateDescriptorSets(device_.device(), &allocInfo, &descriptor) != VK_SUCCESS) {
        return false;
    }

    return true;
}

void VklDescriptorPool::freeDescriptors(std::vector<VkDescriptorSet> &descriptors) const {
    vkFreeDescriptorSets(device_.device(), descriptorPool, static_cast<uint32_t>(descriptors.size()),
                         descriptors.data());
}

void VklDescriptorPool::resetPool() {
    vkResetDescriptorPool(device_.device(), descriptorPool, 0);
}

VklDescriptorWriter::VklDescriptorWriter(VklDescriptorSetLayout &setLayout, VklDescriptorPool &pool)
    : setLayout_(setLayout), pool_(pool) {
    writes_.reserve(5);
}

VklDescriptorWriter &VklDescriptorWriter::writeBuffer(uint32_t binding, VkDescriptorBufferInfo *bufferInfo) {
    auto &bindingDescription = setLayout_.bindings_[binding];

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.descriptorType = bindingDescription.descriptorType;
    write.dstBinding = binding;
    write.pBufferInfo = bufferInfo;
    write.descriptorCount = 1;

    writes_.push_back(write);
    return *this;
}

VklDescriptorWriter &VklDescriptorWriter::writeImage(uint32_t binding, VkDescriptorImageInfo *imageInfo) {
    auto &bindingDescription = setLayout_.bindings_[binding];

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.descriptorType = bindingDescription.descriptorType;
    write.dstBinding = binding;
    write.pImageInfo = imageInfo;
    write.descriptorCount = 1;

    writes_.push_back(write);
    return *this;
}

bool VklDescriptorWriter::build(VkDescriptorSet &set) {
    bool success = pool_.allocateDescriptor(setLayout_.getDescriptorSetLayout(), set);
    if (!success) {
        return false;
    }
    overwrite(set);
    return true;
}

void VklDescriptorWriter::overwrite(VkDescriptorSet &set) {
    for (auto &write : writes_) {
        write.dstSet = set;
    }
    vkUpdateDescriptorSets(pool_.device_.device(), writes_.size(), writes_.data(), 0, nullptr);
}
