#pragma once

#include <functional>
#include <memory>

#include "vkl/core/vkl_framebuffer.hpp"
#include "vkl/core/vkl_render_pass.hpp"

#include "render_context.hpp"
#include "render_pass_reflection.hpp"

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

    virtual void compile(RenderContext *render_context, const CompileData &compile_data) {
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
                VkAttachmentDescription attachmentDescription{
                    .format = f.get_format(),
                    .samples = f.get_vk_sample_count_flag_bits(),
                    .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
                    .storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                    .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                    .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                };

                attachments.push_back(attachmentDescription);

                input_refs.push_back({.attachment = static_cast<uint32_t>(attachments.size() - 1),
                                      .layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
            }

            // output attachments
            if (f.get_visibility() == RenderPassReflection::Field::Visibility::Input) {
                VkAttachmentDescription attachmentDescription{.format = f.get_format(),
                                                              .samples = f.get_vk_sample_count_flag_bits(),
                                                              .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD,
                                                              .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
                                                              .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                              .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
                                                              .initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                                              .finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

                attachments.push_back(attachmentDescription);

                if (f.get_type() == RenderPassReflection::Field::Type::TextureDepth) {
                    attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                    attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

                    has_depth_write = true;

                    depth_resolve_ref = {.attachment = static_cast<uint32_t>(attachments.size() - 1),
                                         .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
                } else {
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
                            .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
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
                            .layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
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

        // create VkFrameBuffer

        std::vector<VkImageView> attachmentImageViews;

        for (auto f : compile_data.connected_resources) {
            // only handle image textures
            if (f.get_type() != RenderPassReflection::Field::Type::TextureDepth and
                f.get_type() != RenderPassReflection::Field::Type::Texture2D)
                continue;

            Resource *resource = render_context->resource_manager.get_resource(f.get_name());

            if (resource->get_type() == Resource::Type::ColorTexture) {
                auto color_texture = static_cast<ColorTextureResource *>(resource);
                attachmentImageViews.push_back(color_texture->getTexture()->getTextureImageView());
                if (color_texture->get_resolved_texture()) {
                    attachmentImageViews.push_back(color_texture->get_resolved_texture()->getTextureImageView());
                }
            }
        }

        vkl_frame_buffer = std::make_unique<VklFramebuffer>(
            device_, vkl_render_pass->renderPass, static_cast<uint32_t>(attachmentImageViews.size()),
            attachmentImageViews.data(), render_context->get_width(), render_context->get_height());
    }

    virtual void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) = 0;

    virtual RenderPassReflection render_pass_reflect() = 0;

  protected:
    RenderPass(VklDevice &device) : device_(device) {
    }

    VklDevice &device_;
    std::string name_;

    std::function<void(void)> record_function_;

    std::unique_ptr<VklRenderPass> vkl_render_pass;
    std::unique_ptr<VklFramebuffer> vkl_frame_buffer;

    friend class RenderGraph;
    friend class RenderGraphCompiler;
};

} // namespace RenderGraph
} // namespace GraphicsLab