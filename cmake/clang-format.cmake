find_program(CLANG_FORMAT clang-format)
find_package(PythonInterp REQUIRED)

if (CLANG_FORMAT)
    add_custom_target( LAB_clang_format
        COMMAND
        ${PYTHON_EXECUTABLE} scripts\\clang-formatter.py
        WORKING_DIRECTORY
        ${CMAKE_SOURCE_DIR}
    )
endif()