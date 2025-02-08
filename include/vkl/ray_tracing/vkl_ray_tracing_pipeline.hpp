#pragma once

#include "vkl/core/vkl_device.hpp"
#include "vkl/core/vkl_pipeline.hpp"
#include "vkl/core/vkl_shader_module.hpp"
namespace vkl {

struct VklRayTracingPipeline: public VklPipeline {
    private:
    std::vector<VkShaderModule> shader_modules;
    std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups;

    void createRayTracingPipeline(std::vector<VklShaderModuleInfo> shader_modules);

public:
    VkPipeline rayTracingPipeline;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;

    struct VklRayTracingPipelineShaderPath {
        std::string ray_gen_shader_path;
        std::string miss_shader_path;
        std::string hit_shader_path;
    };

    VklRayTracingPipeline(VklDevice& device, std::vector<VklShaderModuleInfo> shaderInfos, VklRayTracingPipelineShaderPath shaderPath): VklPipeline(device) {
        VkDescriptorSetLayoutBinding acceleration_structure_layout_binding{};
	    acceleration_structure_layout_binding.binding         = 0;
	    acceleration_structure_layout_binding.descriptorType  = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	    acceleration_structure_layout_binding.descriptorCount = 1;
	    acceleration_structure_layout_binding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

	    VkDescriptorSetLayoutBinding result_image_layout_binding{};
	    result_image_layout_binding.binding         = 1;
	    result_image_layout_binding.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
	    result_image_layout_binding.descriptorCount = 1;
	    result_image_layout_binding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

	    VkDescriptorSetLayoutBinding uniform_buffer_binding{};
	    uniform_buffer_binding.binding         = 2;
	    uniform_buffer_binding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	    uniform_buffer_binding.descriptorCount = 1;
	    uniform_buffer_binding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

	    std::vector<VkDescriptorSetLayoutBinding> bindings = {
	        acceleration_structure_layout_binding,
	        result_image_layout_binding,
	        uniform_buffer_binding};

	    VkDescriptorSetLayoutCreateInfo layout_info{};
	    layout_info.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	    layout_info.bindingCount = static_cast<uint32_t>(bindings.size());
	    layout_info.pBindings    = bindings.data();
	    vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr, &descriptorSetLayout);

	    VkPipelineLayoutCreateInfo pipeline_layout_create_info{};
	    pipeline_layout_create_info.sType          = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	    pipeline_layout_create_info.setLayoutCount = 1;
	    pipeline_layout_create_info.pSetLayouts    = &descriptorSetLayout;

	    vkCreatePipelineLayout(device_.device(), &pipeline_layout_create_info, nullptr, &pipelineLayout);

	    // Ray tracing shaders require SPIR-V 1.4, so we need to set the appropriate target environment for the glslang compiler
//	    vkb::GLSLCompiler::set_target_environment(glslang::EShTargetSpv, glslang::EShTargetSpv_1_4);

	    /*
	        Setup ray tracing shader groups
	        Each shader group points at the corresponding shader in the pipeline
	    */
	    std::vector<VkPipelineShaderStageCreateInfo> shader_stages;

	    // Ray generation group
	    {
		    shader_stages.push_back(loadShader(shaderPath.ray_gen_shader_path, VK_SHADER_STAGE_RAYGEN_BIT_KHR));
		    VkRayTracingShaderGroupCreateInfoKHR raygen_group_ci{};
		    raygen_group_ci.sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
		    raygen_group_ci.type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
		    raygen_group_ci.generalShader      = static_cast<uint32_t>(shader_stages.size()) - 1;
		    raygen_group_ci.closestHitShader   = VK_SHADER_UNUSED_KHR;
		    raygen_group_ci.anyHitShader       = VK_SHADER_UNUSED_KHR;
		    raygen_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
            shader_modules.push_back(shader_stages.back().module);
		    shaderGroups.push_back(raygen_group_ci);
	    }

	    // Ray miss group
	    {
		    shader_stages.push_back(loadShader(shaderPath.miss_shader_path, VK_SHADER_STAGE_MISS_BIT_KHR));
		    VkRayTracingShaderGroupCreateInfoKHR miss_group_ci{};
		    miss_group_ci.sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
		    miss_group_ci.type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
		    miss_group_ci.generalShader      = static_cast<uint32_t>(shader_stages.size()) - 1;
		    miss_group_ci.closestHitShader   = VK_SHADER_UNUSED_KHR;
		    miss_group_ci.anyHitShader       = VK_SHADER_UNUSED_KHR;
		    miss_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
            shader_modules.push_back(shader_stages.back().module);
		    shaderGroups.push_back(miss_group_ci);
	    }

	    // Ray closest hit group
	    {
		    shader_stages.push_back(loadShader(shaderPath.hit_shader_path, VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR));
		    VkRayTracingShaderGroupCreateInfoKHR closes_hit_group_ci{};
		    closes_hit_group_ci.sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
		    closes_hit_group_ci.type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
		    closes_hit_group_ci.generalShader      = VK_SHADER_UNUSED_KHR;
		    closes_hit_group_ci.closestHitShader   = static_cast<uint32_t>(shader_stages.size()) - 1;
		    closes_hit_group_ci.anyHitShader       = VK_SHADER_UNUSED_KHR;
		    closes_hit_group_ci.intersectionShader = VK_SHADER_UNUSED_KHR;
            shader_modules.push_back(shader_stages.back().module);
		    shaderGroups.push_back(closes_hit_group_ci);
	    }

	    /*
	        Create the ray tracing pipeline
	    */
	    VkRayTracingPipelineCreateInfoKHR raytracing_pipeline_create_info{};
	    raytracing_pipeline_create_info.sType                        = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
	    raytracing_pipeline_create_info.stageCount                   = static_cast<uint32_t>(shader_stages.size());
	    raytracing_pipeline_create_info.pStages                      = shader_stages.data();
	    raytracing_pipeline_create_info.groupCount                   = static_cast<uint32_t>(shaderGroups.size());
	    raytracing_pipeline_create_info.pGroups                      = shaderGroups.data();
	    raytracing_pipeline_create_info.maxPipelineRayRecursionDepth = 1;
	    raytracing_pipeline_create_info.layout                       = pipelineLayout;
	    vkCreateRayTracingPipelinesKHR(device_.device(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &raytracing_pipeline_create_info, nullptr, &rayTracingPipeline);
    }

    ~VklRayTracingPipeline();

    VklRayTracingPipeline(VklRayTracingPipeline const&) = delete;
    VklRayTracingPipeline &operator=(VklRayTracingPipeline const&) = delete;
};

}