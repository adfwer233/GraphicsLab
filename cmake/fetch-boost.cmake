include(FetchContent)

set(BOOST_INCLUDE_LIBRARIES math numeric)
set(BOOST_ENABLE_CMAKE ON)
set(BOOST_VERSION 1.86.0)

FetchContent_Declare(
        Boost
        URL https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-cmake.7z
        DOWNLOAD_EXTRACT_TIMESTAMP true
        USES_TERMINAL_DOWNLOAD TRUE
        GIT_PROGRESS TRUE
        DOWNLOAD_NO_EXTRACT FALSE
)

FetchContent_MakeAvailable(Boost)

file(GLOB Boost_INCLUDE_DIRS LIST_DIRECTORIES true ${Boost_SOURCE_DIR}/libs/*/include ${Boost_SOURCE_DIR}/libs)