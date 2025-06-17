#pragma once

#include <vector>

namespace vkl {

struct SoftwareRasterizer {
    using Framebuffer = std::vector<uint8_t>;
    using ZBuffer = std::vector<float>;

    explicit SoftwareRasterizer(VklDevice &device, int width, int height)
        : device_(device), width_(width), height_(height) {
        create_resources();
    }

    void clear() {
        std::ranges::fill(framebuffer_, 0);
        std::ranges::fill(zbuffer_, 1.0f);
    }

    void create_staging_buffer(VkDeviceSize size, VkBuffer &buffer, VkDeviceMemory &memory) {
        VkBufferCreateInfo bufferInfo = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        vkCreateBuffer(device_.device(), &bufferInfo, nullptr, &buffer);

        VkMemoryRequirements memReqs;
        vkGetBufferMemoryRequirements(device_.device(), buffer, &memReqs);

        VkMemoryAllocateInfo allocInfo = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = memReqs.size;
        allocInfo.memoryTypeIndex = device_.findMemoryType(
            memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkAllocateMemory(device_.device(), &allocInfo, nullptr, &memory);
        vkBindBufferMemory(device_.device(), buffer, memory, 0);
    }

    void rasterize_mesh(Mesh3D &mesh, glm::mat4 mvp, glm::vec3 view_point, glm::vec3 light_point) {
        for (auto indices : mesh.indices) {
            auto a = mesh.vertices[indices.i];
            auto b = mesh.vertices[indices.j];
            auto c = mesh.vertices[indices.k];

            a.position.y *= -1.0f;
            b.position.y *= -1.0f;
            c.position.y *= -1.0f;

            rasterize_triangle({a, b, c}, framebuffer_, zbuffer_, mvp, light_point, view_point);
        }
    }

    void copy_result_to_image(VkCommandBuffer command_buffer, VkImage image) {
        VkDeviceSize bufferSize = framebuffer_.size();

        // Copy framebuffer data to staging buffer
        void *data;
        vkMapMemory(device_.device(), stagingBufferMemory_, 0, bufferSize, 0, &data);
        std::memcpy(data, framebuffer_.data(), bufferSize);
        vkUnmapMemory(device_.device(), stagingBufferMemory_);

        // Insert barrier: layout to TRANSFER_DST_OPTIMAL
        VkImageMemoryBarrier resultColor2Dst = VklImageUtils::ColorToTransferDstBarrier(image);
        vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &resultColor2Dst);

        // Buffer to image copy
        VkBufferImageCopy copyRegion{};
        copyRegion.bufferOffset = 0;
        copyRegion.bufferRowLength = 0;
        copyRegion.bufferImageHeight = 0;
        copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copyRegion.imageSubresource.mipLevel = 0;
        copyRegion.imageSubresource.baseArrayLayer = 0;
        copyRegion.imageSubresource.layerCount = 1;
        copyRegion.imageOffset = {0, 0, 0};
        copyRegion.imageExtent = {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_), 1};

        vkCmdCopyBufferToImage(command_buffer, stagingBuffer_, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                               &copyRegion);

        // Transition back to fragment shader usage or present
        VkImageMemoryBarrier resultTranGen2Dst = VklImageUtils::transferDstToColorBarrier(image);
        vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
                             0, nullptr, 0, nullptr, 1, &resultTranGen2Dst);
    }

  private:
    uint8_t linear_to_srgb(float x) {
        x = glm::clamp(x, 0.0f, 1.0f);
        if (x <= 0.0031308f)
            return static_cast<uint8_t>(x * 12.92f * 255.0f + 0.5f);
        else
            return static_cast<uint8_t>((1.055f * std::pow(x, 1.0f / 2.4f) - 0.055f) * 255.0f + 0.5f);
    }

    void create_resources() {
        framebuffer_.resize(width_ * height_ * 4);
        zbuffer_.resize(width_ * height_ * 4);

        create_staging_buffer(framebuffer_.size() * sizeof(Framebuffer::value_type), stagingBuffer_,
                              stagingBufferMemory_);
    }

    glm::vec3 barycentric(glm::vec2 p, glm::vec2 a, glm::vec2 b, glm::vec2 c) {
        glm::vec2 v0 = b - a, v1 = c - a, v2 = p - a;
        float d00 = glm::dot(v0, v0);
        float d01 = glm::dot(v0, v1);
        float d11 = glm::dot(v1, v1);
        float d20 = glm::dot(v2, v0);
        float d21 = glm::dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;
        return glm::vec3(u, v, w);
    }

    void put_pixel(Framebuffer &fb, int x, int y, glm::vec3 color) {
        int idx = 4 * (y * width_ + x);
        fb[idx + 0] = linear_to_srgb(color.r);
        fb[idx + 1] = linear_to_srgb(color.g);
        fb[idx + 2] = linear_to_srgb(color.b);
        fb[idx + 3] = 255;
    }

    float edge_func(glm::vec2 a, glm::vec2 b, glm::vec2 c) {
        return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
    }

    glm::vec3 shade_blinn_phong(const glm::vec3& position, const glm::vec3& normal, const glm::vec3& color,
                            const glm::vec3& light_pos, const glm::vec3& view_pos, float shininess = 32.0f) {
        glm::vec3 N = glm::normalize(normal);
        glm::vec3 L = glm::normalize(light_pos - position);
        glm::vec3 V = glm::normalize(view_pos - position);
        glm::vec3 H = glm::normalize(L + V);

        float diff = glm::max(glm::dot(N, L), 0.0f);
        float spec = glm::pow(glm::max(glm::dot(N, H), 0.0f), shininess);

        glm::vec3 ambient = 0.1f * color;
        glm::vec3 diffuse = 0.6f * diff * color;
        glm::vec3 specular = 0.3f * spec * glm::vec3(1.0f); // white specular

        return ambient + diffuse + specular;
    }

    void rasterize_triangle(const std::array<Vertex3D, 3> &tri, Framebuffer &fb, ZBuffer &zb, const glm::mat4 &MVP, const glm::vec3 &light_pos, const glm::vec3 &view_pos) {
        glm::vec4 clip[3];
        for (int i = 0; i < 3; ++i)
            clip[i] = MVP * glm::vec4(tri[i].position, 1.0f);

        glm::vec3 ndc[3];
        for (int i = 0; i < 3; ++i)
            ndc[i] = glm::vec3(clip[i]) / clip[i].w;

        glm::vec2 screen[3];
        for (int i = 0; i < 3; ++i) {
            screen[i].x = (ndc[i].x * 0.5f + 0.5f) * width_;
            screen[i].y = (1.0f - (ndc[i].y * 0.5f + 0.5f)) * height_;
        }

        auto edge = [](glm::vec2 a, glm::vec2 b, glm::vec2 c) {
            return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
        };

        float area = edge(screen[0], screen[1], screen[2]);
        if (area == 0.0f)
            return;

        float minX = std::floor(std::min({screen[0].x, screen[1].x, screen[2].x}));
        float maxX = std::ceil(std::max({screen[0].x, screen[1].x, screen[2].x}));
        float minY = std::floor(std::min({screen[0].y, screen[1].y, screen[2].y}));
        float maxY = std::ceil(std::max({screen[0].y, screen[1].y, screen[2].y}));

        for (int y = std::max(0, int(minY)); y < std::min(height_, int(maxY)); ++y) {
            for (int x = std::max(0, int(minX)); x < std::min(width_, int(maxX)); ++x) {
                glm::vec2 p(x + 0.5f, y + 0.5f);

                float w0 = edge(screen[1], screen[2], p);
                float w1 = edge(screen[2], screen[0], p);
                float w2 = edge(screen[0], screen[1], p);

                // Use top-left rule (non-strict >= 0), or adjust if needed
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                    float invArea = 1.0f / area;
                    float alpha = w0 * invArea;
                    float beta = w1 * invArea;
                    float gamma = w2 * invArea;

                    float depth = alpha * ndc[0].z + beta * ndc[1].z + gamma * ndc[2].z;
                    int idx = y * width_ + x;
                    if (depth < zb[idx]) {
                        zb[idx] = depth;

                        glm::vec3 position = alpha * tri[0].position + beta  * tri[1].position + gamma * tri[2].position;
                        glm::vec3 normal = glm::normalize(alpha * tri[0].normal + beta * tri[1].normal + gamma * tri[2].normal);
                        glm::vec3 color = alpha * tri[0].color + beta * tri[1].color + gamma * tri[2].color;
                        glm::vec3 shaded = shade_blinn_phong(position, normal, color, light_pos, view_pos);

                        put_pixel(fb, x, y, shaded);
                    }
                }
            }
        }
    }

    VklDevice &device_;

    Framebuffer framebuffer_;
    ZBuffer zbuffer_;

    VkBuffer stagingBuffer_;
    VkDeviceMemory stagingBufferMemory_;

    int width_, height_;
};

} // namespace vkl