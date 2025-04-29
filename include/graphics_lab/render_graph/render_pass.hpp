#pragma once

#include <array>
#include <functional>
#include <memory>

#include "vkl/core/vkl_framebuffer.hpp"
#include "vkl/core/vkl_render_pass.hpp"

#include "render_context.hpp"
#include "render_pass_reflection.hpp"

#include <vkl/core/vkl_descriptor.hpp>

namespace GraphicsLab {
namespace RenderGraph {

/**
 * @brief: Data passed to the render pass when execute.
 */
struct RenderPassExecuteData {
    /**
     * pointers to render resources.
     */
};

/**
 * @brief Render Graphics Descriptor in Graphics Lab Render Graph
 */
struct RenderPass {

    struct CompileData {
        /**
         * data used to compile render pass.
         */

        RenderPassReflection connected_resources;
    };

    virtual ~RenderPass() = default;

    const std::string_view get_name() {
        return name_;
    }

    void set_extent(uint32_t width, uint32_t height) {
        width_ = width;
        height_ = height;
    }

    virtual void compile(RenderContext *render_context, const CompileData &compile_data) {
        /**
         * create descriptors for internal resouces
         */

        bool has_internal_resources = false;
        for (auto f : compile_data.connected_resources) {
            if (f.get_visibility() == RenderPassReflection::Field::Visibility::Internal) {
                has_internal_resources = true;
            }
        }

        if (has_internal_resources) {
            spdlog::info("RenderPass {} has been compiled with internal resources", name_);

            auto descriptorSetLayoutBuilder = VklDescriptorSetLayout::Builder(device_);
            int bind_index = 0;
            for (auto f : compile_data.connected_resources) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Internal) {
                    descriptorSetLayoutBuilder.addBinding(bind_index,
                                                          VkDescriptorType::VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                          VkShaderStageFlagBits::VK_SHADER_STAGE_FRAGMENT_BIT);
                    bind_index++;
                }
            }
            descriptorSetLayout = descriptorSetLayoutBuilder.build();

            int pool_size = 10;
            descriptorPool =
                VklDescriptorPool::Builder(device_)
                    .setMaxSets(VklSwapChain::MAX_FRAMES_IN_FLIGHT * pool_size)
                    .addPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VklSwapChain::MAX_FRAMES_IN_FLIGHT * pool_size)
                    .build();
            VklDescriptorWriter writer(*descriptorSetLayout.get(), *descriptorPool);

            bind_index = 0;
            for (auto f : compile_data.connected_resources) {
                if (f.get_visibility() == RenderPassReflection::Field::Visibility::Internal) {
                    if (auto tex = dynamic_cast<ColorTextureResource *>(
                            render_context->resource_manager.get_resource(f.get_name()))) {
                        auto image_info = tex->getTexture()->descriptorInfo();
                        writer.writeImage(bind_index, &image_info);
                    }
                    bind_index++;
                }
            }
            writer.build(descriptorSet);
        }

        /**
         * create VkRenderPass
         */
        std::vector<VkAttachmentDescription> attachments;
        std::vector<VkAttachmentReference> input_refs;
        std::vector<VkAttachmentReference> output_refs;

        std::vector<VkAttachmentReference> resolve_refs;
        VkAttachmentReference depth_attachment_ref;
        VkAttachmentReference depth_resolve_ref;

        bool has_depth_write = false;

        for (auto f : compile_data.connected_resources) {
            // only handle image textures
            if (f.get_type() != RenderPassReflection::Field::Type::TextureDepth and
                f.get_type() != RenderPassReflection::Field::Type::Texture2D)
                continue;

            // input attachments
            if (f.get_visibility() == RenderPassReflection::Field::Visibility::Input) {
                VkAttachmentDescription attachmentDescription{.format = f.get_format(),
                                                              .samples = VK_SAMPLE_COUNT_1_BIT,
                                                              .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
                                                              .storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                                              .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                              .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                                              .initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                                              .finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

                attachments.push_back(attachmentDescription);

                input_refs.push_back({.attachment = static_cast<uint32_t>(attachments.size() - 1),
                                      .layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
            }

            // output attachments
            if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                VkAttachmentDescription attachmentDescription{.format = f.get_format(),
                                                              .samples = f.get_vk_sample_count_flag_bits(),
                                                              .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                              .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
                                                              .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                              .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                                              .initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                                              .finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

                if (f.get_type() == RenderPassReflection::Field::Type::TextureDepth) {
                    attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                    attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                    attachmentDescription.format = device_.findSupportedFormat(
                        {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT},
                        VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
                    attachments.push_back(attachmentDescription);

                    has_depth_write = true;

                    depth_attachment_ref = {.attachment = static_cast<uint32_t>(attachments.size() - 1),
                                            .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
                } else {
                    attachments.push_back(attachmentDescription);
                    output_refs.push_back({.attachment = static_cast<uint32_t>(attachments.size() - 1),
                                           .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL});
                }

                if (f.need_to_resolve()) {
                    if (f.get_type() == RenderPassReflection::Field::Type::TextureDepth) { // depth attachment
                        VkAttachmentDescription resolveAttachment = {};
                        resolveAttachment.format = attachmentDescription.format; // Format of your resolve image
                        resolveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;       // No MSAA for resolve image
                        resolveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                        resolveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                        resolveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                        resolveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                        resolveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                        resolveAttachment.finalLayout = attachmentDescription.finalLayout;

                        attachments.push_back(resolveAttachment);
                        depth_resolve_ref = {
                            .attachment = static_cast<uint32_t>(attachments.size() - 1),
                            .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                        };
                    } else { // color attachment
                        VkAttachmentDescription resolveAttachment = {};
                        resolveAttachment.format = attachmentDescription.format; // Format of your resolve image
                        resolveAttachment.samples = VK_SAMPLE_COUNT_1_BIT;       // No MSAA for resolve image
                        resolveAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                        resolveAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                        resolveAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                        resolveAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                        resolveAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                        resolveAttachment.finalLayout = attachmentDescription.finalLayout;

                        attachments.push_back(resolveAttachment);
                        resolve_refs.push_back({
                            .attachment = static_cast<uint32_t>(attachments.size() - 1),
                            .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                        });
                    }
                }
            }
        }

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
            dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
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
            dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
            dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
        }

        VkSubpassDescription subpassDescription{.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                .inputAttachmentCount = static_cast<uint32_t>(input_refs.size()),
                                                .pInputAttachments = input_refs.data(),
                                                .colorAttachmentCount = static_cast<uint32_t>(output_refs.size()),
                                                .pColorAttachments = output_refs.data(),
                                                .pResolveAttachments = resolve_refs.data()};

        std::vector<VkAttachmentReference> depth_refs;
        if (has_depth_write) {
            depth_refs = {depth_attachment_ref, depth_resolve_ref};
            subpassDescription.pDepthStencilAttachment = depth_refs.data();
        }

        VkRenderPassCreateInfo renderPassCreateInfo{};
        renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassCreateInfo.pAttachments = attachments.data();
        renderPassCreateInfo.subpassCount = 1;
        renderPassCreateInfo.pSubpasses = &subpassDescription;
        renderPassCreateInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassCreateInfo.pDependencies = dependencies.data();

        vkl_render_pass = std::make_unique<VklRenderPass>(device_, renderPassCreateInfo);

        spdlog::info("Compiling render pass {}, render pass attachment count {}", name_, attachments.size());
        // create VkFrameBuffer

        std::vector<VkImageView> attachmentImageViews;

        for (auto f : compile_data.connected_resources) {
            // only handle image textures
            if (f.get_type() != RenderPassReflection::Field::Type::TextureDepth and
                f.get_type() != RenderPassReflection::Field::Type::Texture2D)
                continue;

            Resource *resource = render_context->resource_manager.get_resource(f.get_name());

            if (f.get_visibility() == RenderPassReflection::Field::Visibility::Output) {
                if (resource->get_type() == Resource::Type::ColorTexture) {
                    auto color_texture = reinterpret_cast<ColorTextureResource *>(resource);
                    attachmentImageViews.push_back(color_texture->getTexture()->getTextureImageView());
                    if (color_texture->get_resolved_texture()) {
                        attachmentImageViews.push_back(color_texture->get_resolved_texture()->getTextureImageView());
                    }
                }
            } else if (f.get_visibility() == RenderPassReflection::Field::Visibility::Input) {
                if (resource->get_type() == Resource::Type::ColorTexture) {
                    auto color_texture = reinterpret_cast<ColorTextureResource *>(resource);
                    if (color_texture->get_resolved_texture()) {
                        attachmentImageViews.push_back(color_texture->get_resolved_texture()->getTextureImageView());
                    } else {
                        attachmentImageViews.push_back(color_texture->getTexture()->getTextureImageView());
                    }
                }
            }
        }

        spdlog::info("Compiling render pass {}, framebuffer attachment count {}", name_, attachmentImageViews.size());

        vkl_frame_buffer = std::make_unique<VklFramebuffer>(device_, vkl_render_pass->renderPass,
                                                            static_cast<uint32_t>(attachmentImageViews.size()),
                                                            attachmentImageViews.data(), width_, height_);
    }

    virtual void post_compile(RenderContext *render_context) = 0;

    virtual void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) = 0;

    virtual RenderPassReflection render_pass_reflect() = 0;

    void set_name(const std::string &name) {
        name_ = name;
    }

  protected:
    void begin_render_pass(VkCommandBuffer commandBuffer) {
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = vkl_render_pass->renderPass;
        renderPassInfo.framebuffer = vkl_frame_buffer->framebuffer;

        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = {2048, 2048};

        /**
         * @todo: auto generate clear values
         */
        std::array<VkClearValue, 3> clearValues{};
        clearValues[0].color = clear_color_;
        clearValues[1].color = clear_color_;
        clearValues[2].depthStencil = {1.0f, 0};
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(2048);
        viewport.height = static_cast<float>(2048);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        VkRect2D scissor{{0, 0}, {2048, 2048}};
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    }

    void end_render_pass(VkCommandBuffer commandBuffer) {
        vkCmdEndRenderPass(commandBuffer);
    }

    explicit RenderPass(VklDevice &device, const VkClearColorValue clear_color = {0.01f, 0.01f, 0.01f, 1.0f})
        : device_(device), clear_color_(clear_color) {
    }

    VklDevice &device_;
    std::string name_;

    std::function<void(void)> record_function_;

    std::unique_ptr<VklRenderPass> vkl_render_pass;
    std::unique_ptr<VklFramebuffer> vkl_frame_buffer;

    uint32_t width_ = 2048, height_ = 2048;

    VkClearColorValue clear_color_;

    std::unique_ptr<VklDescriptorSetLayout> descriptorSetLayout = nullptr;
    std::unique_ptr<VklDescriptorPool> descriptorPool = nullptr;
    VkDescriptorSet descriptorSet = nullptr;

    friend class RenderGraphCompiler;
};

} // namespace RenderGraph
} // namespace GraphicsLab