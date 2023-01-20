#include <app.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>

int main(int argc, char const *argv[])
{
    auto console = spdlog::stdout_color_mt("console");
    auto err_logger = spdlog::stderr_color_mt("stderr");
    spdlog::set_pattern("%^%L%$> %v");

    // Init GLFW
    if (!glfwInit()) {
        spdlog::error("GLFW3 failed to initialize!");
        exit(EXIT_FAILURE);
    }

    App app;
    glfwMakeContextCurrent(app.window);

    // Get and display version information
	std::string renderer((char*)glGetString(GL_RENDERER));
	std::string version((char*)glGetString(GL_VERSION));
    spdlog::info("OpenGL version supported: {}", version);
    spdlog::info("Renderer: {}", renderer);
    spdlog::info("Max texture size: {}", GL_MAX_TEXTURE_SIZE);
    
    // Render loop
	app.run();
    return 0;
}
