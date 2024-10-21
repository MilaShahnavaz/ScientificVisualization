ivw_module(TNM067Lab2)

set(HEADER_FILES
    include/inviwo/tnm067lab2/tnm067lab2module.h
    include/inviwo/tnm067lab2/tnm067lab2moduledefine.h
    include/inviwo/tnm067lab2/processors/hydrogengenerator.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/tnm067lab2module.cpp
    src/processors/hydrogengenerator.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
    #glsl/tnm067lab2processor.frag
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    tests/unittests/hydrogen-test.cpp
    tests/unittests/tnm067lab2-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
ivw_folder(inviwo-module-tnm067lab2 TNM067)

add_custom_command( TARGET inviwo-unittests-tnm067lab2 POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy
                        $<TARGET_RUNTIME_DLLS:inviwo-unittests-tnm067lab2>
                        $<TARGET_FILE_DIR:inviwo-unittests-tnm067lab2>
                    COMMAND_EXPAND_LISTS)
