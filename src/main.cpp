#include <app.hpp>
#include <gleq.h>
#include <spdlog/sinks/stdout_color_sinks.h>

int main(int argc, char const *argv[])
{
    // Setup logging
    auto console = spdlog::stdout_color_mt("console");
    auto err_logger = spdlog::stderr_color_mt("stderr");
    spdlog::set_pattern("%^%L%$> %v");
    spdlog::set_level(spdlog::level::debug);

    // Init GLFW
    if (!glfwInit()) {
        spdlog::error("GLFW3 failed to initialize!");
        return -1;
    }


    gleqInit();

    App app;

    // Get and display version information
	std::string renderer((char*)glGetString(GL_RENDERER));
	std::string version((char*)glGetString(GL_VERSION));
    spdlog::info("OpenGL {}, {}", version, renderer);
    
    // Render loop
	app.run();
    return 0;
}
