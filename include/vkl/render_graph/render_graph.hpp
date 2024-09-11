#pragma once

#include <format>
#include <iostream>
#include <optional>
#include <vector>

#include "../core/vkl_renderer.hpp"
#include "language/meta_programming/type_list.hpp"

#include "vkl/core/vkl_framebuffer.hpp"
#include "vkl/core/vkl_render_pass.hpp"
#include "vkl/core/vkl_texture.hpp"

#include "language/coroutine/generator.hpp"

#include "vkl/core/vkl_shader_module.hpp"
#include "vkl/system/render_system/render_system_base.hpp"

#include "spdlog/spdlog.h"

struct RenderGraphRenderPass {};
struct RenderGraphComputePass {};
struct RenderGraphTransferPass {};
struct RenderGraphSubgraphPass {};
struct RenderGraphImagePresentPass {};
struct RenderGraphFrameSyncBeginPass {};
struct RenderGraphFrameSyncEndPass {};

using RenderGraphPassTypeList =
    MetaProgramming::TypeList<RenderGraphRenderPass, RenderGraphComputePass, RenderGraphSubgraphPass,
                              RenderGraphImagePresentPass, RenderGraphTransferPass, RenderGraphFrameSyncBeginPass,
                              RenderGraphFrameSyncEndPass>;

struct RenderGraphTextureAttachment {};
struct RenderGraphStorageBufferAttachment {};

using RenderGraphAttachmentTypeList =
    MetaProgramming::TypeList<RenderGraphTextureAttachment, RenderGraphStorageBufferAttachment>;

template <typename T>
concept RenderGraphPass = MetaProgramming::TypeListFunctions::IsAnyOf<RenderGraphPassTypeList, T>::value;

template <typename T>
concept RenderGraphAttachment = MetaProgramming::TypeListFunctions::IsAnyOf<RenderGraphAttachmentTypeList, T>::value;

struct RenderGraphObjectsBase {
    std::string name;
};

struct RenderGraphAttachmentDescriptorBase : public RenderGraphObjectsBase {
    virtual ~RenderGraphAttachmentDescriptorBase() = default;
};

template <RenderGraphAttachment T>
struct RenderGraphAttachmentDescriptor : public RenderGraphAttachmentDescriptorBase {};

template <RenderGraphAttachment T> struct RenderGraphAttachmentInstance : public RenderGraphObjectsBase {};

struct RenderGraphAttachmentBase : public RenderGraphObjectsBase {
    virtual ~RenderGraphAttachmentBase() = default;
};

struct RenderGraphPassDescriptorBase : public RenderGraphObjectsBase {
    using AttachmentDescriptorVectorsType = RenderGraphAttachmentTypeList::monad<
        RenderGraphAttachmentDescriptor>::to_ptr::monad<std::vector>::to<std::tuple>;

    AttachmentDescriptorVectorsType input_descriptors;
    AttachmentDescriptorVectorsType output_descriptors;

    template <RenderGraphAttachment AttachmentType>
    std::vector<RenderGraphAttachmentDescriptor<AttachmentType> *> get_input_attachment_vector() {
        return std::get<
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value>(
            input_descriptors);
    }

    template <RenderGraphAttachment AttachmentType>
    std::vector<RenderGraphAttachmentDescriptor<AttachmentType> *> get_output_attachment_vector() {
        return std::get<
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value>(
            input_descriptors);
    }

    virtual ~RenderGraphPassDescriptorBase() = default;
};

template <RenderGraphPass T> struct RenderGraphPassDescriptor : public RenderGraphPassDescriptorBase {};

template <RenderGraphPass T> struct RenderGraphPassInstance : public RenderGraphObjectsBase {};

template <RenderGraphAttachment T> struct RenderGraphAttachmentDerived : public RenderGraphAttachmentBase {
    RenderGraphAttachmentDescriptor<T> *descriptor_p{};
    std::vector<RenderGraphAttachmentInstance<T> *> instances;

    ~RenderGraphAttachmentDerived() override {
        for (auto instance : instances)
            delete instance;
    }
};

template <> struct RenderGraphAttachmentInstance<RenderGraphTextureAttachment> {
    std::unique_ptr<VklTexture> texture;
};

template <> struct RenderGraphAttachmentDerived<RenderGraphTextureAttachment> : public RenderGraphAttachmentBase {
    RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment> *descriptor_p{};
    std::vector<RenderGraphAttachmentInstance<RenderGraphTextureAttachment> *> instances;

    ~RenderGraphAttachmentDerived() override {
        for (auto instance : instances)
            delete instance;
    }

    auto getImguiTextures() {
        std::vector<VkDescriptorSet> result;
        for (auto instance : instances) {
            auto tex = ImGui_ImplVulkan_AddTexture(instance->texture->getTextureSampler(),
                                                   instance->texture->getTextureImageView(),
                                                   instance->texture->getImageLayout());
            result.push_back(tex);
        }
        return result;
    }
};

struct RenderGraphPassBase : public RenderGraphObjectsBase {
    RenderGraphPassDescriptorBase *descriptor_p;
    std::vector<RenderGraphAttachmentBase *> input_attachments;
    std::vector<RenderGraphAttachmentBase *> output_attachments;

    using RenderGraphAttachmentVectorsType =
        RenderGraphAttachmentTypeList::monad<RenderGraphAttachmentDerived>::to_ptr::monad<std::vector>::to<std::tuple>;
    RenderGraphAttachmentVectorsType input_attachment_vectors;
    RenderGraphAttachmentVectorsType output_attachment_vectors;

    template <RenderGraphAttachment AttachmentType>
    std::vector<RenderGraphAttachmentDerived<AttachmentType> *> &get_input_attachment_vector() {
        constexpr size_t index =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value;
        return std::get<index>(input_attachment_vectors);
    }

    template <RenderGraphAttachment AttachmentType>
    std::vector<RenderGraphAttachmentDerived<AttachmentType> *> &get_output_attachment_vector() {
        constexpr size_t index =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value;
        return std::get<index>(output_attachment_vectors);
    }
};

template <RenderGraphPass T> struct RenderGraphPassDerived : public RenderGraphPassBase {
    std::vector<RenderGraphPassInstance<T> *> instances;
};

template <> struct RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment> {
    std::string name;

    uint32_t width, height;

    VkFormat format;

    enum class AttachmentType {
        ColorAttachment,
        DepthAttachment
    };

    AttachmentType type;

    VkClearValue clearValue;

    bool isSwapChain{false};
    bool clear{false};
    bool input_flag{false};
    bool output_flag{false};
};

template <> struct RenderGraphPassInstance<RenderGraphRenderPass> {
    std::unique_ptr<VklFramebuffer> framebuffer;

    std::vector<RenderGraphAttachmentInstance<RenderGraphTextureAttachment>> in;
    std::vector<RenderGraphAttachmentInstance<RenderGraphTextureAttachment>> out;
};

template <> struct RenderGraphPassDerived<RenderGraphRenderPass> : public RenderGraphPassBase {
    std::vector<RenderGraphPassInstance<RenderGraphRenderPass> *> instances;
    RenderGraphPassDescriptor<RenderGraphRenderPass> *descriptor_p;
    std::unique_ptr<VklRenderPass> renderPass;
    bool is_submit_pass = false;

    std::optional<std::function<void(VkCommandBuffer, uint32_t)>> recordFunction = std::nullopt;

    ~RenderGraphPassDerived() {
        for (auto instance : instances)
            delete instance;
        for (auto [name, render_system] : render_system_cache)
            delete render_system;
    }

    template <typename RenderSystemType>
    RenderSystemType *getRenderSystem(VklDevice &device, const std::string &name,
                                      std::vector<VklShaderModuleInfo> &&shader_info) {
        if (render_system_cache.contains(name)) {
            auto casted_ptr = dynamic_cast<RenderSystemType *>(render_system_cache[name]);
            if (casted_ptr != nullptr) {
                return casted_ptr;
            } else {
                throw std::runtime_error("render system type error");
            }
        } else {
            auto render_system = new RenderSystemType(device, renderPass->renderPass, shader_info);
            render_system_cache[name] = dynamic_cast<BaseRenderSystem *>(render_system);
            return render_system;
        }
    }

  private:
    std::unordered_map<std::string, BaseRenderSystem *> render_system_cache;
};

struct RenderGraph;
template <> struct RenderGraphPassDerived<RenderGraphSubgraphPass> : public RenderGraphPassBase {};

template <> struct RenderGraphPassDescriptor<RenderGraphRenderPass> : public RenderGraphPassDescriptorBase {
    uint32_t width, height;
    bool is_submit_pass = false;

    std::vector<RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment> *> inTextureAttachmentDescriptors;
    std::vector<RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment> *> outTextureAttachmentDescriptors;

    friend class RenderGraph;
};

struct RenderGraphDescriptor;
template <> struct RenderGraphPassDescriptor<RenderGraphSubgraphPass> : public RenderGraphPassDescriptorBase {
    RenderGraphDescriptor *renderGraphDescriptor;
};

template <> struct RenderGraphPassDescriptor<RenderGraphImagePresentPass> : public RenderGraphPassDescriptorBase {
    RenderGraphPassDescriptor<RenderGraphImagePresentPass> &setImage(VkImageView presentImageView) {
        presentImageView_ = presentImageView;
        return *this;
    }

  private:
    VkImageView presentImageView_;
    friend class RenderGraph;
};

struct RenderGraphDescriptor {
    using RenderGraphPassDescriptorPtrVectorsType =
        RenderGraphPassTypeList::monad<RenderGraphPassDescriptor>::to_ptr::monad<std::vector>::to<std::tuple>;
    using RenderGraphAttachmentDescriptorPtrVectorsType = RenderGraphAttachmentTypeList::monad<
        RenderGraphAttachmentDescriptor>::to_ptr::monad<std::vector>::to<std::tuple>;

    RenderGraphPassDescriptorPtrVectorsType renderGraphPassDescriptorPtrVectors;
    RenderGraphAttachmentDescriptorPtrVectorsType renderGraphAttachmentDescriptorPtrVectors;

    std::vector<RenderGraphPassDescriptorBase *> passDescriptors;
    std::vector<RenderGraphAttachmentDescriptorBase *> attachmentDescriptors;

    template <size_t index> Generator<RenderGraphPassDescriptorBase *> all_pass_descriptors_detail() {
        for (auto pass_desc : std::get<index>(renderGraphPassDescriptorPtrVectors))
            co_yield (RenderGraphPassDescriptorBase *) pass_desc;
        if constexpr (index > 0) {
            for (auto pass_desc : all_pass_descriptors_detail<index - 1>()) {
                co_yield pass_desc;
            }
        }
    }

    template <size_t index> Generator<RenderGraphAttachmentDescriptorBase *> all_attachment_descriptors_detail() {
        for (auto att_desc : std::get<index>(renderGraphAttachmentDescriptorPtrVectors))
            co_yield (RenderGraphAttachmentDescriptorBase *) att_desc;
        if constexpr (index > 0) {
            for (auto att_desc : all_attachment_descriptors_detail<index - 1>()) {
                co_yield att_desc;
            }
        }
    }

    Generator<RenderGraphPassDescriptorBase *> all_pass_descriptors() {
        return all_pass_descriptors_detail<RenderGraphPassTypeList::size - 1>();
    }

    Generator<RenderGraphAttachmentDescriptorBase *> all_attachment_descriptors() {
        return all_attachment_descriptors_detail<RenderGraphAttachmentTypeList::size - 1>();
    }

    template <RenderGraphPass PassType> RenderGraphPassDescriptor<PassType> *pass(const std::string &name) {
        auto node = new RenderGraphPassDescriptor<PassType>();
        node->name = name;
        constexpr uint32_t index =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphPassTypeList, PassType>::value;
        std::get<index>(renderGraphPassDescriptorPtrVectors).push_back(node);
        return node;
    }

    template <RenderGraphAttachment AttachmentType>
    RenderGraphAttachmentDescriptor<AttachmentType> *attachment(const std::string &name) {
        auto edge = new RenderGraphAttachmentDescriptor<AttachmentType>();
        edge->name = name;

        constexpr uint32_t index =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value;
        std::get<index>(renderGraphAttachmentDescriptorPtrVectors).push_back(edge);
        return edge;
    }

    template <RenderGraphAttachment AttachmentType>
    RenderGraphAttachmentDescriptor<AttachmentType> *getAttachment(const std::string &name) {
        constexpr uint32_t index =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value;
        for (auto att : std::get<index>(renderGraphAttachmentDescriptorPtrVectors)) {
            if (att->name == name)
                return att;
        }
    }
};

struct RenderGraph {

    VklDevice &device_;
    std::unique_ptr<VklSwapChain> swapChain_;

    uint32_t instance_n_;
    std::vector<VkCommandBuffer> commandBuffers;

    std::vector<RenderGraphPassBase *> passes;
    std::vector<RenderGraphAttachmentBase *> attachments;

    uint32_t currentFrame = 0;
    uint32_t current_image_index = 0;

    VkExtent2D extent_;

    uint32_t beginFrame() {
        auto result = swapChain_->acquireNextImage(&current_image_index);

        if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        return currentFrame;
    }

    void endFrame() {
        currentFrame = (currentFrame + 1) % 2;
    }

    using RenderGraphPassObjectPtrVectorsType =
        RenderGraphPassTypeList::monad<RenderGraphPassDerived>::to_ptr::monad<std::vector>::to<std::tuple>;
    using RenderGraphAttachmentObjectPtrVectorsType =
        RenderGraphAttachmentTypeList::monad<RenderGraphAttachmentDerived>::to_ptr::monad<std::vector>::to<std::tuple>;

    RenderGraphPassObjectPtrVectorsType renderGraphPassObjectPtrVectors;
    RenderGraphAttachmentObjectPtrVectorsType renderGraphAttachmentObjectPtrVectors;

    template <size_t id> Generator<RenderGraphPassBase *> all_passes_generator_detail() {
        for (auto pass : std::get<id>(renderGraphPassObjectPtrVectors)) {
            co_yield (RenderGraphPassBase *) pass;
        }

        if constexpr (id > 0) {
            for (auto pass : all_passes_generator_detail<id - 1>()) {
                co_yield pass;
            }
        }
    }

    Generator<RenderGraphPassBase *> all_passes_generator() {
        return all_passes_generator_detail<RenderGraphPassTypeList::size - 1>();
    }

    template <RenderGraphPass PassType> Generator<RenderGraphPassDerived<PassType> *> passes_generator() {
        for (auto pass :
             std::get<MetaProgramming::TypeListFunctions::IndexOf<RenderGraphPassTypeList, PassType>::value>(
                 renderGraphPassObjectPtrVectors)) {
            co_yield pass;
        }
    }

    template <size_t id> Generator<RenderGraphAttachmentBase *> all_attachments_generator_detail() {
        for (auto att : std::get<id>(renderGraphAttachmentObjectPtrVectors)) {
            co_yield (RenderGraphAttachmentBase *) att;
        }

        if constexpr (id > 0) {
            for (auto att : all_attachments_generator_detail<id - 1>()) {
                co_yield att;
            }
        }
    }

    Generator<RenderGraphAttachmentBase *> all_attachments_generator() {
        return all_attachments_generator_detail<RenderGraphAttachmentTypeList::size - 1>();
    }

    template <RenderGraphAttachment AttachmentType>
    Generator<RenderGraphAttachmentDerived<AttachmentType> *> attachments_generator() {
        for (auto att : std::get<
                 MetaProgramming::TypeListFunctions::IndexOf<RenderGraphAttachmentTypeList, AttachmentType>::value>(
                 renderGraphAttachmentObjectPtrVectors)) {
            co_yield att;
        }
    }

    RenderGraphDescriptor *renderGraphDescriptor_;

    static RenderGraphPassBase *pass_desc_to_obj(RenderGraphPassDescriptorBase *descriptor_p) {
        RenderGraphPassBase *res = nullptr;

        auto for_loop_body = [&]<RenderGraphPass PassType>(PassType passType) {
            if (dynamic_cast<RenderGraphPassDescriptor<PassType> *>(descriptor_p) != nullptr) {
                auto obj = new RenderGraphPassDerived<PassType>();
                obj->descriptor_p = dynamic_cast<RenderGraphPassDescriptor<PassType> *>(descriptor_p);
                res = obj;
            }
        };

        auto for_loop = [&]<RenderGraphPass... PassTypes>(MetaProgramming::TypeList<PassTypes...>) {
            (for_loop_body(PassTypes()), ...);
        };

        for_loop(RenderGraphPassTypeList());

        return res;
    }

    static RenderGraphAttachmentBase *attachment_desc_to_obj(RenderGraphAttachmentDescriptorBase *descriptor_p) {
        RenderGraphAttachmentBase *res = nullptr;

        auto for_loop_body = [&]<RenderGraphAttachment AttachmentTypes>(AttachmentTypes attachmentTypes) {
            if (dynamic_cast<RenderGraphAttachmentDescriptor<AttachmentTypes> *>(descriptor_p) != nullptr) {
                auto obj = new RenderGraphAttachmentDerived<AttachmentTypes>();
                obj->descriptor_p = dynamic_cast<RenderGraphAttachmentDescriptor<AttachmentTypes> *>(descriptor_p);
                res = obj;
            }
        };

        auto for_loop = [&]<RenderGraphAttachment... AttachmentTypes>(MetaProgramming::TypeList<AttachmentTypes...>) {
            (for_loop_body(AttachmentTypes()), ...);
        };

        for_loop(RenderGraphAttachmentTypeList());

        return res;
    }

    template <RenderGraphAttachment AttachmentType>
    [[nodiscard]] RenderGraphAttachmentDerived<AttachmentType> *getAttachment(const std::string &name) {
        for (auto att : attachments_generator<AttachmentType>()) {
            if (att->name == name) {
                return att;
            }
        }
        return nullptr;
    }

    template <RenderGraphPass PassType>
    [[nodiscard]] RenderGraphPassDerived<PassType> *getPass(const std::string &name) {
        for (auto pass : passes_generator<PassType>()) {
            if (pass->name == name) {
                return pass;
            }
        }
        return nullptr;
    }

    void expandSubgraphDescriptor(RenderGraphDescriptor *renderGraphDescriptor) {
        constexpr uint32_t id =
            MetaProgramming::TypeListFunctions::IndexOf<RenderGraphPassTypeList, RenderGraphSubgraphPass>::value;
        for (auto pass_desc : std::get<id>(renderGraphDescriptor->renderGraphPassDescriptorPtrVectors)) {
            constructor_copy_pass_detail<RenderGraphPassTypeList::size - 1>(pass_desc->renderGraphDescriptor);
            constructor_copy_attachment_detail<RenderGraphAttachmentTypeList::size - 1>(
                pass_desc->renderGraphDescriptor);
        }
    }

    template <size_t id> void constructor_copy_pass_detail(RenderGraphDescriptor *renderGraphDescriptor) {
        using T = typename MetaProgramming::TypeListFunctions::KthOf<RenderGraphPassTypeList, id>::type;
        if constexpr (std::is_same_v<T, RenderGraphSubgraphPass>) {
            constructor_copy_pass_detail<id - 1>(renderGraphDescriptor);
            return;
        }

        auto &obj_vector = std::get<id>(renderGraphPassObjectPtrVectors);
        for (auto pass_desc : std::get<id>(renderGraphDescriptor->renderGraphPassDescriptorPtrVectors)) {
            auto obj_ptr = new RenderGraphPassDerived<T>();
            obj_ptr->descriptor_p = pass_desc;
            obj_ptr->name = pass_desc->name;
            obj_vector.push_back(obj_ptr);
        }

        if constexpr (id > 0) {
            constructor_copy_pass_detail<id - 1>(renderGraphDescriptor);
        }
    }

    template <size_t id> void constructor_copy_attachment_detail(RenderGraphDescriptor *renderGraphDescriptor) {
        auto &obj_vector = std::get<id>(renderGraphAttachmentObjectPtrVectors);
        for (auto att_desc : std::get<id>(renderGraphDescriptor->renderGraphAttachmentDescriptorPtrVectors)) {
            auto obj_ptr = new RenderGraphAttachmentDerived<
                typename MetaProgramming::TypeListFunctions::KthOf<RenderGraphAttachmentTypeList, id>::type>();
            obj_ptr->descriptor_p = att_desc;
            obj_ptr->name = att_desc->name;
            obj_vector.push_back(obj_ptr);
        }

        if constexpr (id > 0) {
            constructor_copy_attachment_detail<id - 1>(renderGraphDescriptor);
        }
    };

    void recreateSwapChain(VkExtent2D extent) {
        vkDeviceWaitIdle(device_.device());

        if (swapChain_ == nullptr) {
            swapChain_ = std::make_unique<VklSwapChain>(device_, extent);
        } else {
            std::shared_ptr<VklSwapChain> oldSwapChain = std::move(swapChain_);
            swapChain_ = std::make_unique<VklSwapChain>(device_, extent, oldSwapChain);

            if (!oldSwapChain->compareSwapFormats(*swapChain_.get())) {
                throw std::runtime_error("Swap chain image(or depth) format has changed!");
            }
        }
    }

    void createCommandBuffers() {
        /**
         * create command buffers for the render graph
         */
        commandBuffers.resize(instance_n_);

        VkCommandBufferAllocateInfo allocateInfo;
        allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandPool = device_.getCommandPool();
        allocateInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());
        allocateInfo.pNext = nullptr;

        if (vkAllocateCommandBuffers(device_.device(), &allocateInfo, commandBuffers.data()) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate command buffers!");
        }
    }

    explicit RenderGraph(VklDevice &device, VkExtent2D extent, RenderGraphDescriptor *renderGraphDescriptor)
        : device_(device), extent_(extent) {
        renderGraphDescriptor_ = renderGraphDescriptor;
        recreateSwapChain(extent);

        instance_n_ = swapChain_->imageCount();

        /**
         * create node/edge objects
         */
        expandSubgraphDescriptor(renderGraphDescriptor);
        constructor_copy_pass_detail<RenderGraphPassTypeList::size - 1>(renderGraphDescriptor);
        constructor_copy_attachment_detail<RenderGraphAttachmentTypeList::size - 1>(renderGraphDescriptor);

        for (auto renderPass : passes_generator<RenderGraphRenderPass>()) {
            renderPass->is_submit_pass = renderPass->descriptor_p->is_submit_pass;
        }

        /**
         * copy attachent informations to the pass objects
         */

        for (auto pass : passes_generator<RenderGraphRenderPass>()) {
            auto &pass_texture_input = pass->get_input_attachment_vector<RenderGraphTextureAttachment>();
            for (auto input_desc : pass->descriptor_p->inTextureAttachmentDescriptors) {
                auto att = getAttachment<RenderGraphTextureAttachment>(input_desc->name);
                pass_texture_input.push_back(att);
            }

            auto &pass_texture_output = pass->get_output_attachment_vector<RenderGraphTextureAttachment>();
            for (auto output_desc : pass->descriptor_p->outTextureAttachmentDescriptors) {
                auto att = getAttachment<RenderGraphTextureAttachment>(output_desc->name);
                pass_texture_output.push_back(att);
            }
        }

        createCommandBuffers();
    }

    ~RenderGraph() {
        for (auto edge : attachments_generator<RenderGraphTextureAttachment>())
            delete edge;
        for (auto node : passes_generator<RenderGraphRenderPass>())
            delete node;
    }

    void createLayouts() {
        /**
         * create render pass for each `RenderGraphRenderPass` node in render graph
         */

        for (auto renderNode : passes_generator<RenderGraphRenderPass>()) {
            std::vector<VkAttachmentDescription> node_attachments;
            std::vector<VkAttachmentReference> input_refs;
            std::vector<VkAttachmentReference> output_refs;

            VkAttachmentReference depth_attachment_ref;

            uint32_t attachment_index = 0;

            bool has_depth_write = false;

            // input attachments
            for (auto input_desc : renderNode->descriptor_p->inTextureAttachmentDescriptors) {
                VkAttachmentDescription attachmentDescription{
                    .format = input_desc->format,
                    .samples = VK_SAMPLE_COUNT_1_BIT,
                    .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
                    .storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                    .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                    .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                };

                if (input_desc->type == std::decay_t<decltype(*input_desc)>::AttachmentType::DepthAttachment) {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                } else {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                }

                node_attachments.push_back(attachmentDescription);

                input_refs.push_back(
                    {.attachment = attachment_index, .layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
            }

            // output attachments
            for (auto output_desc : renderNode->descriptor_p->outTextureAttachmentDescriptors) {
                VkAttachmentDescription attachmentDescription{
                    .format = output_desc->format,
                    .samples = VK_SAMPLE_COUNT_1_BIT,
                    .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
                    .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
                    .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                    .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                };

                if (output_desc->clear) {
                    attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                    attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                } else if (not output_desc->isSwapChain) {
                    attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
                    attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
                } else {
                    attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                    attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                }

                // initial layout
                if (output_desc->type == std::decay_t<decltype(*output_desc)>::AttachmentType::DepthAttachment) {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                } else if (not output_desc->isSwapChain) {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                } else {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                }

                // final layout
                if (output_desc->type == std::decay_t<decltype(*output_desc)>::AttachmentType::DepthAttachment) {
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                } else if (output_desc->isSwapChain) {
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
                } else {
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                }

                if (output_desc->type == std::decay_t<decltype(*output_desc)>::AttachmentType::DepthAttachment) {
                    if (has_depth_write) {
                        throw std::runtime_error("Cannot write to multiple depth attachments!");
                    }
                    has_depth_write = true;
                    node_attachments.push_back(attachmentDescription);
                    depth_attachment_ref = {
                        .attachment = attachment_index++,
                        .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    };
                } else {
                    node_attachments.push_back(attachmentDescription);
                    output_refs.push_back({
                        .attachment = attachment_index++,
                        .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    });
                }
            }

            VkSubpassDescription subpassDescription{
                .pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS,
                .inputAttachmentCount = static_cast<uint32_t>(input_refs.size()),
                .pInputAttachments = input_refs.data(),
                .colorAttachmentCount = static_cast<uint32_t>(output_refs.size()),
                .pColorAttachments = output_refs.data(),
            };

            if (has_depth_write)
                subpassDescription.pDepthStencilAttachment = &depth_attachment_ref;

            std::vector<VkSubpassDependency> dependencies;
            dependencies.resize(2);

            dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
            dependencies[0].dstSubpass = 0;
            dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
            dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
            dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

            if (has_depth_write) {
                dependencies.resize(3);

                dependencies[1].srcSubpass = 0;
                dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
                dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                dependencies[1].dstStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
                dependencies[1].srcAccessMask =
                    VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                dependencies[1].dstAccessMask =
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

                dependencies[2].srcSubpass = 0;
                dependencies[2].dstSubpass = VK_SUBPASS_EXTERNAL;
                dependencies[2].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
                dependencies[2].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
                dependencies[2].srcAccessMask =
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                dependencies[2].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
                dependencies[2].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
            } else {
                dependencies[1].srcSubpass = 0;
                dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
                dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
                dependencies[1].srcAccessMask =
                    VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
                dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
            }

            VkRenderPassCreateInfo renderPassCreateInfo{};
            renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
            renderPassCreateInfo.attachmentCount = static_cast<uint32_t>(node_attachments.size());
            renderPassCreateInfo.pAttachments = node_attachments.data();
            renderPassCreateInfo.subpassCount = 1;
            renderPassCreateInfo.pSubpasses = &subpassDescription;
            renderPassCreateInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
            renderPassCreateInfo.pDependencies = dependencies.data();

            renderNode->renderPass = std::make_unique<VklRenderPass>(device_, renderPassCreateInfo);
        }
        /**
         * create compute pass for each 'RenderGraphComputePass' node in render graph
         */
    }

    void createInstances() {
        // create render resources.

        for (auto pass : passes_generator<RenderGraphRenderPass>()) {
            for (int i = 0; i < instance_n_; i++) {
                pass->instances.emplace_back(new RenderGraphPassInstance<RenderGraphRenderPass>());
            }
        }

        for (auto edge : attachments_generator<RenderGraphTextureAttachment>()) {
            for (int i = 0; i < instance_n_; i++) {
                edge->instances.emplace_back(new RenderGraphAttachmentInstance<RenderGraphTextureAttachment>());
            }
        }

        for (int i = 0; i < instance_n_; i++) {
            /**
             * create texture instances
             */
            for (auto edge : attachments_generator<RenderGraphTextureAttachment>()) {
                spdlog::info("create instance for texture {}, instance id {}", edge->name, i);

                VkImageUsageFlags imageUsage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
                VkImageLayout layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                if (edge->descriptor_p->input_flag)
                    imageUsage |= VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;

                if (edge->descriptor_p->type ==
                    RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment>::AttachmentType::DepthAttachment) {
                    imageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                    layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                }

                VkFormat format = edge->descriptor_p->format;

                edge->instances[i]->texture = std::move(std::make_unique<VklTexture>(
                    device_, edge->descriptor_p->width, edge->descriptor_p->height, 4, imageUsage, layout, format));
            }

            /**
             * create storage buffer instances
             */
            for (auto edge : attachments_generator<RenderGraphStorageBufferAttachment>()) {
            }

            /**
             * create framebuffer for nodes
             */

            for (auto node : passes_generator<RenderGraphRenderPass>()) {
                std::vector<VkImageView> attachmentImageViews;

                for (auto edge : node->get_input_attachment_vector<RenderGraphTextureAttachment>()) {
                    attachmentImageViews.push_back(edge->instances[i]->texture->getTextureImageView());
                }

                for (auto edge : node->get_output_attachment_vector<RenderGraphTextureAttachment>()) {
                    if (edge->descriptor_p->isSwapChain) {
                        attachmentImageViews.push_back(swapChain_->getImageView(i));
                    } else {
                        attachmentImageViews.push_back(edge->instances[i]->texture->getTextureImageView());
                    }
                }

                node->instances[i]->framebuffer = std::make_unique<VklFramebuffer>(
                    device_, node->renderPass->renderPass, static_cast<uint32_t>(attachmentImageViews.size()),
                    attachmentImageViews.data(), node->descriptor_p->width, node->descriptor_p->height);
            }
        }
    }

    VkResult render(VkCommandBuffer commandBuffer, uint32_t frame_index) {
        // begin command buffer

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

        if (vkBeginCommandBuffer(commandBuffers[frame_index], &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording command buffer!");
        }

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        VkRect2D scissor{{0, 0}, swapChain_->getSwapChainExtent()};

        /**
         * Execute render passes
         */
        for (auto render_pass : passes_generator<RenderGraphRenderPass>()) {
            uint32_t width = render_pass->descriptor_p->width;
            uint32_t height = render_pass->descriptor_p->height;

            if (render_pass->is_submit_pass) {
                auto swap_chain_extent = swapChain_->getSwapChainExtent();
                width = swap_chain_extent.width;
                height = swap_chain_extent.height;
            }

            viewport.width = width;
            viewport.height = height;
            scissor.extent = {width, height};

            vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
            vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

            VkRenderPassBeginInfo renderPassInfo{};
            renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            renderPassInfo.renderPass = render_pass->renderPass->renderPass;
            renderPassInfo.framebuffer = render_pass->instances[frame_index]->framebuffer->framebuffer;

            renderPassInfo.renderArea.offset = {0, 0};
            renderPassInfo.renderArea.extent = {width, height};

            if (render_pass->is_submit_pass) {
                renderPassInfo.renderPass = swapChain_->getRenderPass();
                renderPassInfo.framebuffer = swapChain_->getFrameBuffer(frame_index);
            }

            std::array<VkClearValue, 2> clearValues{};
            clearValues[0].color = {0.01f, 0.01f, 0.01f, 1.0f};
            clearValues[1].depthStencil = {1.0f, 0};
            renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
            renderPassInfo.pClearValues = clearValues.data();

            vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

            if (render_pass->recordFunction.has_value()) {
                render_pass->recordFunction.value()(commandBuffer, frame_index);
            }

            vkCmdEndRenderPass(commandBuffer);
        }

        // end command buffer
        vkEndCommandBuffer(commandBuffers[frame_index]);
        std::vector<VkCommandBuffer> commandBufferToSubmit = {commandBuffers[frame_index]};
        auto result = swapChain_->submitCommandBuffers(commandBufferToSubmit, &frame_index);

        return result;
    }
};