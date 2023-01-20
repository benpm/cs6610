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

    // Init GLFW window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "CS6610", NULL, NULL);
    if (!window) {
        spdlog::error("Could not open GLFW window");
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Get and display version information
	std::string renderer((char*)glGetString(GL_RENDERER));
	std::string version((char*)glGetString(GL_VERSION));
    spdlog::info("OpenGL version supported: {}", version);
    spdlog::info("Renderer: {}", renderer);
    spdlog::info("Max texture size: {}", GL_MAX_TEXTURE_SIZE);
    
    // Render loop
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS
        && !glfwWindowShouldClose(window))
    {
        float t = glfwGetTime();
        vec3 clearColor(std::sin(t * 0.7), std::cos(t * 0.9), std::cos(t * 0.67));
        glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        

        GLenum err = glGetError();
        if (err) {
            spdlog::error("OpenGL error: {}", err);
            break;
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
