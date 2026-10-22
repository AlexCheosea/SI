# The following variables contains the files used by the different stages of the build process.
set(16f690_default_default_XC8_FILE_TYPE_assemble)
set_source_files_properties(${16f690_default_default_XC8_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${16f690_default_default_XC8_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(16f690_default_default_XC8_FILE_TYPE_assemblePreprocess)
set_source_files_properties(${16f690_default_default_XC8_FILE_TYPE_assemblePreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${16f690_default_default_XC8_FILE_TYPE_assemblePreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(16f690_default_default_XC8_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../analog.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../init.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../motor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../uart.c")
set_source_files_properties(${16f690_default_default_XC8_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(16f690_default_default_XC8_FILE_TYPE_link)
set(16f690_default_image_name "default.elf")
set(16f690_default_image_base_name "default")

# The output directory of the final image.
set(16f690_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/16f690")

# The full path to the final image.
set(16f690_default_full_path_to_image ${16f690_default_output_dir}/${16f690_default_image_name})

# Potential output file extensions
set(output_extensions
    .hex
    .hxl
    .mum
    .o
    .sdb
    .sym
    .cmf)
list(TRANSFORM output_extensions PREPEND "${16f690_default_output_dir}/${16f690_default_image_base_name}")
