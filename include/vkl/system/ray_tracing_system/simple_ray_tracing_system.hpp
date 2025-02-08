#pragma once

#include "vkl/ray_tracing/vkl_ray_tracing_pipeline.hpp"

namespace vkl {

struct SimpleRayTracingSystem {

    struct AccelerationStructure {
        VkAccelerationStructureKHR handle;
        uint64_t address;
        std::unique_ptr<vkl::Buffer> buffer;
    };

	struct ScratchBuffer
	{
		uint64_t       device_address;
		VkBuffer       handle;
		VkDeviceMemory memory;
	};

	VklDevice &device;
    std::unique_ptr<VklRayTracingPipeline> pipeline;

    AccelerationStructure bottom_level_acceleration_structure;
    AccelerationStructure top_level_acceleration_structure;

	std::unique_ptr<vkl::Buffer> vertex_buffer;
	std::unique_ptr<vkl::Buffer> index_buffer;

	/*
    Create buffer and allocate memory for a temporary scratch buffer
	*/
	ScratchBuffer create_scratch_buffer(VkDeviceSize size) const {
		ScratchBuffer scratch_buffer{};

		VkBufferCreateInfo buffer_create_info = {};
		buffer_create_info.sType              = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		buffer_create_info.size               = size;
		buffer_create_info.usage              = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
		vkCreateBuffer(device.device(), &buffer_create_info, nullptr, &scratch_buffer.handle);

		VkMemoryRequirements memory_requirements = {};
		vkGetBufferMemoryRequirements(device.device(), scratch_buffer.handle, &memory_requirements);

		VkMemoryAllocateFlagsInfo memory_allocate_flags_info = {};
		memory_allocate_flags_info.sType                     = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
		memory_allocate_flags_info.flags                     = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

		VkMemoryAllocateInfo memory_allocate_info = {};
		memory_allocate_info.sType                = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memory_allocate_info.pNext                = &memory_allocate_flags_info;
		memory_allocate_info.allocationSize       = memory_requirements.size;
		memory_allocate_info.memoryTypeIndex      = device.findMemoryType(memory_requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		vkAllocateMemory(device.device(), &memory_allocate_info, nullptr, &scratch_buffer.memory);
		vkBindBufferMemory(device.device(), scratch_buffer.handle, scratch_buffer.memory, 0);

		VkBufferDeviceAddressInfoKHR buffer_device_address_info{};
		buffer_device_address_info.sType  = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		buffer_device_address_info.buffer = scratch_buffer.handle;
		scratch_buffer.device_address     = vkGetBufferDeviceAddressKHR(device.device(), &buffer_device_address_info);

		return scratch_buffer;
	}

	void delete_scratch_buffer(ScratchBuffer &scratch_buffer) const {
		if (scratch_buffer.memory != VK_NULL_HANDLE) {
			vkFreeMemory(device.device(), scratch_buffer.memory, nullptr);
		}
		if (scratch_buffer.handle != VK_NULL_HANDLE) {
			vkDestroyBuffer(device.device(), scratch_buffer.handle, nullptr);
		}
	}

    void createBottemLevelAccelerationStructure() {
        struct Vertex
		{
			float pos[3];
		};
		std::vector<Vertex> vertices = {
		    {{1.0f, 1.0f, 0.0f}},
		    {{-1.0f, 1.0f, 0.0f}},
		    {{0.0f, -1.0f, 0.0f}}};
		std::vector<uint32_t> indices = {0, 1, 2};

		VkDeviceSize vertex_buffer_size = vertices.size() * sizeof(Vertex);
		VkDeviceSize index_buffer_size  = indices.size() * sizeof(uint32_t);

		// Create buffers for the bottom level geometry
		// For the sake of simplicity we won't stage the vertex data to the GPU memory

		// Note that the buffer usage flags for buffers consumed by the bottom level acceleration structure require special flags
		const VkBufferUsageFlags buffer_usage_flags = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

		vertex_buffer = std::make_unique<vkl::Buffer>(device, vertex_buffer_size, buffer_usage_flags, VMA_MEMORY_USAGE_CPU_TO_GPU);
		vertex_buffer->update(vertices.data(), vertex_buffer_size);
		index_buffer = std::make_unique<vkl::Buffer>(device, index_buffer_size, buffer_usage_flags, VMA_MEMORY_USAGE_CPU_TO_GPU);
		index_buffer->update(indices.data(), index_buffer_size);

		// Setup a single transformation matrix that can be used to transform the whole geometry for a single bottom level acceleration structure
		VkTransformMatrixKHR transform_matrix = {
		    1.0f, 0.0f, 0.0f, 0.0f,
		    0.0f, 1.0f, 0.0f, 0.0f,
		    0.0f, 0.0f, 1.0f, 0.0f};
		std::unique_ptr<vkl::Buffer> transform_matrix_buffer = std::make_unique<vkl::Buffer>(device, sizeof(transform_matrix), buffer_usage_flags, VMA_MEMORY_USAGE_CPU_TO_GPU);
		transform_matrix_buffer->update(&transform_matrix, sizeof(transform_matrix));

		VkDeviceOrHostAddressConstKHR vertex_data_device_address{};
		VkDeviceOrHostAddressConstKHR index_data_device_address{};
		VkDeviceOrHostAddressConstKHR transform_matrix_device_address{};

		vertex_data_device_address.deviceAddress      = get_buffer_device_address(vertex_buffer->get_handle());
		index_data_device_address.deviceAddress       = get_buffer_device_address(index_buffer->get_handle());
		transform_matrix_device_address.deviceAddress = get_buffer_device_address(transform_matrix_buffer->get_handle());

		// The bottom level acceleration structure contains one set of triangles as the input geometry
		VkAccelerationStructureGeometryKHR acceleration_structure_geometry{};
		acceleration_structure_geometry.sType                            = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
		acceleration_structure_geometry.geometryType                     = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
		acceleration_structure_geometry.flags                            = VK_GEOMETRY_OPAQUE_BIT_KHR;
		acceleration_structure_geometry.geometry.triangles.sType         = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
		acceleration_structure_geometry.geometry.triangles.vertexFormat  = VK_FORMAT_R32G32B32_SFLOAT;
		acceleration_structure_geometry.geometry.triangles.vertexData    = vertex_data_device_address;
		acceleration_structure_geometry.geometry.triangles.maxVertex     = 3;
		acceleration_structure_geometry.geometry.triangles.vertexStride  = sizeof(Vertex);
		acceleration_structure_geometry.geometry.triangles.indexType     = VK_INDEX_TYPE_UINT32;
		acceleration_structure_geometry.geometry.triangles.indexData     = index_data_device_address;
		acceleration_structure_geometry.geometry.triangles.transformData = transform_matrix_device_address;

		// Get the size requirements for buffers involved in the acceleration structure build process
		VkAccelerationStructureBuildGeometryInfoKHR acceleration_structure_build_geometry_info{};
		acceleration_structure_build_geometry_info.sType         = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		acceleration_structure_build_geometry_info.type          = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		acceleration_structure_build_geometry_info.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		acceleration_structure_build_geometry_info.geometryCount = 1;
		acceleration_structure_build_geometry_info.pGeometries   = &acceleration_structure_geometry;

		const uint32_t primitive_count = 1;

		VkAccelerationStructureBuildSizesInfoKHR acceleration_structure_build_sizes_info{};
		acceleration_structure_build_sizes_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
		vkGetAccelerationStructureBuildSizesKHR(
		    device.device(),
		    VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		    &acceleration_structure_build_geometry_info,
		    &primitive_count,
		    &acceleration_structure_build_sizes_info);

		// Create a buffer to hold the acceleration structure
		bottom_level_acceleration_structure.buffer = std::make_unique<vkl::Buffer>(
		    device,
		    acceleration_structure_build_sizes_info.accelerationStructureSize,
		    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		    VMA_MEMORY_USAGE_GPU_ONLY);

		// Create the acceleration structure
		VkAccelerationStructureCreateInfoKHR acceleration_structure_create_info{};
		acceleration_structure_create_info.sType  = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
		acceleration_structure_create_info.buffer = bottom_level_acceleration_structure.buffer->get_handle();
		acceleration_structure_create_info.size   = acceleration_structure_build_sizes_info.accelerationStructureSize;
		acceleration_structure_create_info.type   = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		vkCreateAccelerationStructureKHR(device.device(), &acceleration_structure_create_info, nullptr, &bottom_level_acceleration_structure.handle);

		// The actual build process starts here

		// Create a scratch buffer as a temporary storage for the acceleration structure build
		ScratchBuffer scratch_buffer = create_scratch_buffer(acceleration_structure_build_sizes_info.buildScratchSize);

		VkAccelerationStructureBuildGeometryInfoKHR acceleration_build_geometry_info{};
		acceleration_build_geometry_info.sType                     = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		acceleration_build_geometry_info.type                      = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		acceleration_build_geometry_info.flags                     = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		acceleration_build_geometry_info.mode                      = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		acceleration_build_geometry_info.dstAccelerationStructure  = bottom_level_acceleration_structure.handle;
		acceleration_build_geometry_info.geometryCount             = 1;
		acceleration_build_geometry_info.pGeometries               = &acceleration_structure_geometry;
		acceleration_build_geometry_info.scratchData.deviceAddress = scratch_buffer.device_address;

		VkAccelerationStructureBuildRangeInfoKHR acceleration_structure_build_range_info;
		acceleration_structure_build_range_info.primitiveCount                                           = 1;
		acceleration_structure_build_range_info.primitiveOffset                                          = 0;
		acceleration_structure_build_range_info.firstVertex                                              = 0;
		acceleration_structure_build_range_info.transformOffset                                          = 0;
		std::vector<VkAccelerationStructureBuildRangeInfoKHR *> acceleration_build_structure_range_infos = {&acceleration_structure_build_range_info};

		// Build the acceleration structure on the device via a one-time command buffer submission
		// Some implementations may support acceleration structure building on the host (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds
		delete_scratch_buffer(scratch_buffer);

		VkCommandBuffer command_buffer = device.beginSingleTimeCommands();
    	vkCmdBuildAccelerationStructuresKHR(
		    command_buffer,
		    1,
		    &acceleration_build_geometry_info,
		    acceleration_build_structure_range_infos.data());
    	device.endSingleTimeCommands(command_buffer);

		// Get the bottom acceleration structure's handle, which will be used during the top level acceleration build
		VkAccelerationStructureDeviceAddressInfoKHR acceleration_device_address_info{};
		acceleration_device_address_info.sType                 = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
		acceleration_device_address_info.accelerationStructure = bottom_level_acceleration_structure.handle;
		bottom_level_acceleration_structure.address =
		    vkGetAccelerationStructureDeviceAddressKHR(device.device(), &acceleration_device_address_info);
    }

    void createTopLevelAccelerationStructure() {
		VkTransformMatrixKHR transform_matrix = {
		    1.0f, 0.0f, 0.0f, 0.0f,
		    0.0f, 1.0f, 0.0f, 0.0f,
		    0.0f, 0.0f, 1.0f, 0.0f};

		VkAccelerationStructureInstanceKHR acceleration_structure_instance{};
		acceleration_structure_instance.transform                              = transform_matrix;
		acceleration_structure_instance.instanceCustomIndex                    = 0;
		acceleration_structure_instance.mask                                   = 0xFF;
		acceleration_structure_instance.instanceShaderBindingTableRecordOffset = 0;
		acceleration_structure_instance.flags                                  = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
		acceleration_structure_instance.accelerationStructureReference         = bottom_level_acceleration_structure.address;

		std::unique_ptr<vkl::Buffer> instances_buffer = std::make_unique<vkl::Buffer>(device,
	                                                                                  sizeof(VkAccelerationStructureInstanceKHR),
	                                                                                  VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
	                                                                                  VMA_MEMORY_USAGE_CPU_TO_GPU);
		instances_buffer->update(&acceleration_structure_instance, sizeof(VkAccelerationStructureInstanceKHR));

		VkDeviceOrHostAddressConstKHR instance_data_device_address{};
		instance_data_device_address.deviceAddress = get_buffer_device_address(instances_buffer->get_handle());

		// The top level acceleration structure contains (bottom level) instance as the input geometry
		VkAccelerationStructureGeometryKHR acceleration_structure_geometry{};
		acceleration_structure_geometry.sType                              = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
		acceleration_structure_geometry.geometryType                       = VK_GEOMETRY_TYPE_INSTANCES_KHR;
		acceleration_structure_geometry.flags                              = VK_GEOMETRY_OPAQUE_BIT_KHR;
		acceleration_structure_geometry.geometry.instances.sType           = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
		acceleration_structure_geometry.geometry.instances.arrayOfPointers = VK_FALSE;
		acceleration_structure_geometry.geometry.instances.data            = instance_data_device_address;

		// Get the size requirements for buffers involved in the acceleration structure build process
		VkAccelerationStructureBuildGeometryInfoKHR acceleration_structure_build_geometry_info{};
		acceleration_structure_build_geometry_info.sType         = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		acceleration_structure_build_geometry_info.type          = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		acceleration_structure_build_geometry_info.flags         = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		acceleration_structure_build_geometry_info.geometryCount = 1;
		acceleration_structure_build_geometry_info.pGeometries   = &acceleration_structure_geometry;

		const uint32_t primitive_count = 1;

		VkAccelerationStructureBuildSizesInfoKHR acceleration_structure_build_sizes_info{};
		acceleration_structure_build_sizes_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
		vkGetAccelerationStructureBuildSizesKHR(
		    device.device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		    &acceleration_structure_build_geometry_info,
		    &primitive_count,
		    &acceleration_structure_build_sizes_info);

		// Create a buffer to hold the acceleration structure
		top_level_acceleration_structure.buffer = std::make_unique<vkl::Buffer>(
		    device,
		    acceleration_structure_build_sizes_info.accelerationStructureSize,
		    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		    VMA_MEMORY_USAGE_GPU_ONLY);

		// Create the acceleration structure
		VkAccelerationStructureCreateInfoKHR acceleration_structure_create_info{};
		acceleration_structure_create_info.sType  = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
		acceleration_structure_create_info.buffer = top_level_acceleration_structure.buffer->get_handle();
		acceleration_structure_create_info.size   = acceleration_structure_build_sizes_info.accelerationStructureSize;
		acceleration_structure_create_info.type   = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		vkCreateAccelerationStructureKHR(device.device(), &acceleration_structure_create_info, nullptr, &top_level_acceleration_structure.handle);

		// The actual build process starts here

		// Create a scratch buffer as a temporary storage for the acceleration structure build
		ScratchBuffer scratch_buffer = create_scratch_buffer(acceleration_structure_build_sizes_info.buildScratchSize);

		VkAccelerationStructureBuildGeometryInfoKHR acceleration_build_geometry_info{};
		acceleration_build_geometry_info.sType                     = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		acceleration_build_geometry_info.type                      = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		acceleration_build_geometry_info.flags                     = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		acceleration_build_geometry_info.mode                      = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		acceleration_build_geometry_info.dstAccelerationStructure  = top_level_acceleration_structure.handle;
		acceleration_build_geometry_info.geometryCount             = 1;
		acceleration_build_geometry_info.pGeometries               = &acceleration_structure_geometry;
		acceleration_build_geometry_info.scratchData.deviceAddress = scratch_buffer.device_address;

		VkAccelerationStructureBuildRangeInfoKHR acceleration_structure_build_range_info;
		acceleration_structure_build_range_info.primitiveCount                                           = 1;
		acceleration_structure_build_range_info.primitiveOffset                                          = 0;
		acceleration_structure_build_range_info.firstVertex                                              = 0;
		acceleration_structure_build_range_info.transformOffset                                          = 0;
		std::vector<VkAccelerationStructureBuildRangeInfoKHR *> acceleration_build_structure_range_infos = {&acceleration_structure_build_range_info};

		// Build the acceleration structure on the device via a one-time command buffer submission
		// Some implementations may support acceleration structure building on the host (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds
		VkCommandBuffer command_buffer = device.beginSingleTimeCommands();
		vkCmdBuildAccelerationStructuresKHR(
		    command_buffer,
		    1,
		    &acceleration_build_geometry_info,
		    acceleration_build_structure_range_infos.data());
		device.endSingleTimeCommands(command_buffer);
		delete_scratch_buffer(scratch_buffer);

		// Get the top acceleration structure's handle, which will be used to setup it's descriptor
		VkAccelerationStructureDeviceAddressInfoKHR acceleration_device_address_info{};
		acceleration_device_address_info.sType                 = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
		acceleration_device_address_info.accelerationStructure = top_level_acceleration_structure.handle;
		top_level_acceleration_structure.address =
		    vkGetAccelerationStructureDeviceAddressKHR(device.device(), &acceleration_device_address_info);
    }

	uint64_t get_buffer_device_address(VkBuffer buffer) {
    	VkBufferDeviceAddressInfoKHR buffer_device_address_info{};
    	buffer_device_address_info.sType  = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    	buffer_device_address_info.buffer = buffer;
    	return vkGetBufferDeviceAddressKHR(device.device(), &buffer_device_address_info);
    }
};

}