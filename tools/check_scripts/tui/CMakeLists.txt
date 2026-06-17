cmake_minimum_required(VERSION 3.10)
project(sunray_check_tui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add third-party libraries
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/ftxui)

# Add include directories
include_directories(include)

# Shared TUI sources. The build and dependency-check tools use the same UI
# implementation, but keep separate binaries so each entrypoint has a stable
# default mode.
set(SUNRAY_TUI_SOURCES
    src/tui_main.cpp
    src/tui_logic.cpp
    src/tui_interaction.cpp
    src/tui_core.cpp
    src/tui_screen.cpp
    src/tui_terminal.cpp
    src/tui_config.cpp
    # Render group
    src/tui_render/tui_render.cpp
    src/tui_render/tui_render_component.cpp
    src/tui_render/tui_render_buttons.cpp
    src/tui_render/tui_render_keyboard.cpp
    src/tui_render/tui_render_mouse.cpp
    src/tui_render/tui_render_layout.cpp
    src/tui_render/tui_render_cells.cpp
    src/tui_render/tui_render_details.cpp
    src/tui_render/tui_render_debug.cpp
    src/tui_render/tui_highlight_manager.cpp
)

add_executable(sunray_check_tui ${SUNRAY_TUI_SOURCES})

foreach(SUNRAY_TUI_TARGET sunray_check_tui)
    target_link_libraries(${SUNRAY_TUI_TARGET}
        yaml-cpp
        ftxui::screen
        ftxui::dom
        ftxui::component
    )

    target_compile_definitions(${SUNRAY_TUI_TARGET}
        PRIVATE HAVE_YAML_CPP HAVE_FTXUI SUNRAY_TUI_DEFAULT_CHECK_MODE
    )

    set_target_properties(${SUNRAY_TUI_TARGET} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../bin"
    )
endforeach()
