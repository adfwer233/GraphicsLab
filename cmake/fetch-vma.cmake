include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        VulkanMemoryAllocator
        GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
)

FetchContent_MakeAvailable(VulkanMemoryAllocator)