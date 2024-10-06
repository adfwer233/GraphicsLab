#pragma once

#include "spdlog/spdlog.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include "renderdoc_app.h"

#ifndef RENDERDOC_DIR
#define RENDERDOC_DIR "renderdoc/"
#endif

namespace GraphicsLab {

struct RenderDocApi {

#ifdef _WIN32
    inline static HMODULE renderdoc_module;
#else
    inline static void *renderdoc_module;
#endif

    inline static RENDERDOC_API_1_6_0 *renderdoc_api;

    static bool initialize_render_doc() {
        if (renderdoc_module)
            return true;

#ifdef _WIN32
        renderdoc_module = GetModuleHandleA(std::format("{}/{}", RENDERDOC_DIR, "renderdoc.dll").c_str());
#elif defined(ANDROID)
        renderdoc_module = dlopen(RENDERDOC_DIR + "libVkLayer_GLES_RenderDoc.so", RTLD_NOW | RTLD_NOLOAD);
#else
        renderdoc_module = dlopen(RENDERDOC_DIR + "librenderdoc.so", RTLD_NOW | RTLD_NOLOAD);
#endif

        if (!renderdoc_module) {
            spdlog::info("Failed to load RenderDoc,start the application from render doc to use the capture!");
            spdlog::info("The path is {}/{}", RENDERDOC_DIR, "renderdoc.dll");
            return false;
        } else {
            spdlog::info("Started from render doc, press C to capture next frame.");
        }

#ifdef _WIN32
        // Workaround GCC warning about FARPROC mismatch.
        auto *gpa = GetProcAddress(renderdoc_module, "RENDERDOC_GetAPI");
        pRENDERDOC_GetAPI func;
        memcpy(&func, &gpa, sizeof(func));

        if (!func) {
            spdlog::error("Failed to load RENDERDOC_GetAPI function.");
            return false;
        }
#else
        auto *func = reinterpret_cast<pRENDERDOC_GetAPI>(dlsym(renderdoc_module, "RENDERDOC_GetAPI"));
        if (!func) {
            LOGE("Failed to load RENDERDOC_GetAPI function.\n");
            return false;
        }
#endif

        if (!func(eRENDERDOC_API_Version_1_6_0, reinterpret_cast<void **>(&renderdoc_api))) {
            spdlog::error("Failed to obtain RenderDoc 1.6.0 API.");
            return false;
        } else {
            int major, minor, patch;
            renderdoc_api->GetAPIVersion(&major, &minor, &patch);
            spdlog::info("Initialized RenderDoc API {}.{}.{}", major, minor, patch);
        }

        return true;
    }
};

} // namespace GraphicsLab