
#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif

#include <cassert>
#include <functional>
#include <algorithm>
#include <app.hpp>
#define GLEQ_IMPLEMENTATION
#include <gleq.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <spdlog/formatter.h>
#include <physics.hpp>

void APIENTRY GLDebugMessageCallback(
    GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length,
    const GLchar *msg, const void *data)
{
    std::string _source;
    switch (source) {
        case GL_DEBUG_SOURCE_API:               _source = "api"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:     _source = "window"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER:   _source = "shader"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:       _source = "3rd party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:       _source = "app"; break;
        case GL_DEBUG_SOURCE_OTHER:             _source = "UNKNOWN"; break;
        default: _source = "UNKNOWN"; break;
    }

    std::string _type;
    switch (type) {
        case GL_DEBUG_TYPE_ERROR:               _type = "error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: _type = "deprecated"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  _type = "undefined"; break;
        case GL_DEBUG_TYPE_PORTABILITY:         _type = "portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE:         _type = "performance"; break;
        case GL_DEBUG_TYPE_OTHER:               _type = "other"; break;
        case GL_DEBUG_TYPE_MARKER:              _type = "marker"; break;
        default: _type = "UNKNOWN"; break;
    }

    std::string _severity;
    spdlog::level::level_enum lvl;
    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:
            _severity = "high";
            lvl = spdlog::level::warn;
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            _severity = "med";
            lvl = spdlog::level::warn;
            break;
        case GL_DEBUG_SEVERITY_LOW:
            _severity = "low";
            lvl = spdlog::level::debug;
            break;
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            _severity = "notif";
            lvl = spdlog::level::trace;
            break;
        default:
            _severity = "UNKNOWN";
            lvl = spdlog::level::debug;
            break;
    }

    if (severity != GL_DEBUG_SEVERITY_NOTIFICATION) {
        spdlog::log(lvl, "GL[{}|{}|{}] {}", _type, _severity, _source, msg);
    }
}


App::App(cxxopts::ParseResult& args) {
    // Initialize GLFW and Gleq
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    this->window = glfwCreateWindow(
        this->windowSize.x(),
        this->windowSize.y(), "CS6610", NULL, NULL);
    if (!this->window) {
        spdlog::error("Could not open GLFW window");
    }
    gleqTrackWindow(this->window);
    glfwMakeContextCurrent(this->window);
    if (glfwRawMouseMotionSupported()) {
        glfwSetInputMode(this->window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
    } else {
        spdlog::warn("GLFW: raw mouse motion not supported");
    }

    // Init glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
    }

    glfwSwapInterval(1);

    // Initialize imgui
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(this->window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    this->loadingScreen("building shader programs, creating buffers");

    // OpenGL config
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_MULTISAMPLE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glLineWidth(2.0f);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(GLDebugMessageCallback, NULL);

    // Build and bind sky shader program
    bool built = this->skyProg.BuildFiles(
        "resources/shaders/sky.vert",
        "resources/shaders/sky.frag");
    if (!built) {
        spdlog::error("Failed to build sky shader program");
    } else {
        this->skyProg.Bind();
    }
    glGenVertexArrays(1, &this->vaoSky); $gl_err();
    glBindVertexArray(this->vaoSky); $gl_err();
    glCreateBuffers(1, &this->vboSky); $gl_err();
    glBindBuffer(GL_ARRAY_BUFFER, this->vboSky); $gl_err();
    std::vector<Vector3f> quadVerts = {
        {-1.0f, -1.0f, 1.0f - 1e-4f},
        { 1.0f, -1.0f, 1.0f - 1e-4f},
        { 1.0f,  1.0f, 1.0f - 1e-4f},

        { 1.0f,  1.0f, 1.0f - 1e-4f},
        {-1.0f,  1.0f, 1.0f - 1e-4f},
        {-1.0f, -1.0f, 1.0f - 1e-4f},
    };
    glBufferData(GL_ARRAY_BUFFER, quadVerts.size() * sizeof(Vector3f),
        quadVerts.data(), GL_STATIC_DRAW); $gl_err();
    GLuint attrib_vPos = this->skyProg.AttribLocation("vPos"); $gl_err();
    glEnableVertexAttribArray(attrib_vPos); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0u, (void*)0u); $gl_err();
    this->skyTextures.addCubemap("sky", "resources/textures/cubemap");
    glBindBuffer(GL_ARRAY_BUFFER, 0); $gl_err();

    // Build and bind wires shader program
    built = this->wiresProg.BuildFiles(
        "resources/shaders/wires.vert",
        "resources/shaders/wires.frag");
    if (!built) {
        spdlog::error("Failed to build wires shader program");
    } else {
        this->wiresProg.Bind();
    }

    // Setup for wires drawing
    glGenVertexArrays(1, &this->vaoWires); $gl_err();
    glBindVertexArray(this->vaoWires); $gl_err();

    glCreateBuffers(1, &this->vboWires); $gl_err();
    glBindBuffer(GL_ARRAY_BUFFER, this->vboWires); $gl_err();
    std::vector<Vector3f> arrowVerts = {
        {+0.0f, +0.0f, +0.0f},
        {-1.0f, +0.0f, +0.0f},

        {-1.0f, +0.0f, +0.0f},
        {-0.9f, -0.1f, +0.0f},

        {-1.0f, +0.0f, +0.0f},
        {-0.9f, +0.1f, +0.0f},
    };
    glBufferData(GL_ARRAY_BUFFER, arrowVerts.size() * sizeof(Vector3f),
        arrowVerts.data(), GL_STATIC_DRAW); $gl_err();

    glCreateBuffers(1, &this->vboPath); $gl_err();
    glCreateBuffers(1, &this->vboBox); $gl_err();
    glCreateBuffers(1, &this->ssboArrows); $gl_err();
    glCreateBuffers(1, &this->ssboArrowColors); $gl_err();
    
    // Build and bind meshes shader program
    built = this->meshProg.BuildFiles(
        "resources/shaders/basic.vert",
        "resources/shaders/basic.frag");
    if (!built) {
        spdlog::error("Failed to build meshes shader program");
    } else {
        this->meshProg.Bind();
    }

    // Create and bind VAO
    glGenVertexArrays(1, &this->vaoMeshes); $gl_err();
    glBindVertexArray(this->vaoMeshes); $gl_err();

    // Load and construct models
    this->loadingScreen("loading meshes");
    this->meshes.add("resources/models/quad.obj", "", false);
    this->meshes.add("resources/models/teapot.obj");
    this->meshes.add("resources/models/suzanne.obj");
    this->meshes.add("resources/models/sphere.obj");
    this->meshes.createSkyMaterial("resources/textures/cubemap");
    this->meshes.setMaterial("teapot", "sky_cubemap");
    this->meshes.setMaterial("sphere", "sky_cubemap");
    this->meshes.setMaterial("suzanne", "teapot.default");

    // Plane reflection framebuffer, depth renderbuffer, and color texture
    glGenTextures(1, &this->texReflections); $gl_err();
    glBindTexture(GL_TEXTURE_2D, this->texReflections); $gl_err();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, this->windowSize.x(), this->windowSize.y(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr); $gl_err();
    glBindTexture(GL_TEXTURE_2D, 0); $gl_err();

    glGenFramebuffers(1, &this->fboReflections); $gl_err();
    glBindFramebuffer(GL_FRAMEBUFFER, this->fboReflections); $gl_err();
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, this->texReflections, 0); $gl_err();

    glGenRenderbuffers(1, &this->rboDepth); $gl_err();
    glBindRenderbuffer(GL_RENDERBUFFER, this->rboDepth); $gl_err();
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, this->windowSize.x(), this->windowSize.y()); $gl_err();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, this->rboDepth); $gl_err();

    glBindRenderbuffer(GL_RENDERBUFFER, 0); $gl_err();
    glBindFramebuffer(GL_FRAMEBUFFER, 0); $gl_err();

    uMaterial& planeRefl = this->meshes.createBufferMaterial("plane_reflection", this->windowSize.x(), this->windowSize.y());
    planeRefl.shininess = 1000.0f;
    planeRefl.diffuseColor = Vector3f::Zero();
    this->meshes.setMaterial("quad", "plane_reflection");

    // uMaterial& skyRefl = this->meshes.getMaterial("sky_cubemap");
    // skyRefl.shininess = 1000.0f;
    // skyRefl.diffuseColor = Vector3f::Zero();

    // Build meshes / materials
    this->meshes.build(this->meshProg);
    spdlog::info("Loaded meshes");

    // Construct scene
    this->loadingScreen("constructing scene");
    for (size_t i = 0; i < this->objectsToGen; i++) {
        entt::entity e = this->makeModel("sphere");
        Model& model = this->reg.get<Model>(e);

        model.scale = Vector3f::Ones() * rng.range(0.2f, 1.0f);
        model.pos = rng.vec(this->box);

        PhysicsBody& body = this->reg.emplace<PhysicsBody>(e);
        body.pos = model.pos;
    }
    {
        entt::entity e = this->makeModel("teapot");
        Model& model = this->reg.get<Model>(e);

        model.rot.x() = -tau4;
        model.pos.y() = 0.25f;
    }
    {
        entt::entity e = this->makeModel("suzanne");
        Model& model = this->reg.get<Model>(e);

        model.rot.x() = -tau4;

        model.pos.y() = 0.25f;
        model.pos.z() = 1.25f;
        model.pos.x() = 1.25f;
    }
    {
        entt::entity e = this->makeModel("quad");
        Model& model = this->reg.get<Model>(e);

        model.scale = vec3(100.0f);

        this->ePlane = e;
    }

    spdlog::debug("placed {} objects", this->reg.view<Model>().size());

    this->camera.orbitDist(3.0f);
    this->secondaryCamera.pos.z() = 20.0f;
    this->secondaryCamera.projection = Camera::Projection::perspective;

    // Add lights
    this->makeLight(
        Vector3f(0.25f, 0.5f, 0.25f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.9f,
        LightType::directional);
    this->makeLight(
        Vector3f(-0.15f, -0.5f, -0.45f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.05f,
        LightType::directional);
    this->makeLight(
        {0.2f, 0.5f, -0.2f},
        {1.0f, 1.0f, 1.0f},
        2.0f,
        LightType::point);
    this->makeLight(
        {0.4f, 0.9f, 0.4f},
        {1.0f, 1.0f, 1.0f},
        2.0f,
        LightType::point);

    // Create and bind model transforms SSBO
    glGenBuffers(1, &this->ssboModels); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels); $gl_err();
    auto& transformsStorage = this->reg.view<ModelTransform>().storage<ModelTransform>();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        transformsStorage.size() * sizeof(Matrix4f),
        *transformsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    // Create and bind lights SSBO
    glGenBuffers(1, &this->ssboLights); $gl_err();
}

App::~App() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();
}

void App::loadingScreen(const std::string& message) {
    glClearColor(0.08f, 0.08f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(this->windowSize.x(), this->windowSize.y()));
    ImGui::Begin("Loading...", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground
        | ImGuiWindowFlags_NoDecoration);
    ImGui::Text(message.c_str());
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(this->window);
}

void App::onKey(int key, bool pressed) {
    if (pressed) {
        this->pressedKeys.emplace(key);
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(this->window, GL_TRUE);
                break;
            case GLFW_KEY_F6: {
                bool built = this->meshProg.BuildFiles(
                    "resources/shaders/basic.vert",
                    "resources/shaders/basic.frag");
                if (!built) {
                    spdlog::error("Failed to build mesh shader program");
                } else {
                    this->meshProg.Bind();
                    spdlog::info("Rebuilt mesh shader program");
                }
                built = this->skyProg.BuildFiles(
                    "resources/shaders/sky.vert",
                    "resources/shaders/sky.frag");
                if (!built) {
                    spdlog::error("Failed to build sky shader program");
                } else {
                    this->skyProg.Bind();
                    spdlog::info("Rebuilt sky shader program");
                }
            } break;
            case GLFW_KEY_P: {
                if (this->camera.projection == Camera::Projection::perspective) {
                    this->camera.projection = Camera::Projection::orthographic;
                    this->camera.zoom = 2000.0f;
                } else {
                    this->camera.projection = Camera::Projection::perspective;
                    this->camera.orbitDist(2.0f);
                }
            } break;
            case GLFW_KEY_1: {
                this->camera.mode = Camera::Mode::orbit;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            } break;
            case GLFW_KEY_2: {
                this->camera.mode = Camera::Mode::fly;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            } break;
            default:
                break;
        }
    } else {
        this->pressedKeys.erase(key);
        switch (key) {
            default:
                break;
        }
    }
}

void App::onResize(int width, int height) {
    glViewport(0, 0, width, height);
    this->windowSize = {width, height};

    spdlog::debug("Resized window to {},{}", width, height);
}

void App::onClick(int button, bool pressed) {
    switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
            this->mouseLeft = pressed;
            this->camera.dragStart();
            this->secondaryCamera.dragStart();
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            this->mouseRight = pressed;
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            this->mouseMiddle = pressed;
            break;
        default:
            break;
    }
    if (pressed) {
        this->mouseClickStart = this->mousePos;
    } else {
        
    }
}

void App::run() {
    while (!glfwWindowShouldClose(this->window)) {
        this->mouseDeltaPos = this->mousePos;
        this->mouseMoved = false;

        // Handle GLFW events (100 gleqs)
        GLEQevent event;
        while (gleqNextEvent(&event)) {
            switch (event.type) {
                case GLEQ_KEY_PRESSED:
                    this->onKey(event.keyboard.key, true);
                    break;
                case GLEQ_KEY_RELEASED:
                    this->onKey(event.keyboard.key, false);
                    break;
                case GLEQ_CURSOR_MOVED:
                    this->mousePos = {event.pos.x, event.pos.y};
                    this->mouseMoved = true;
                    break;
                case GLEQ_FRAMEBUFFER_RESIZED:
                    this->onResize(event.size.width, event.size.height);
                    break;
                case GLEQ_WINDOW_MOVED:
                    this->lastFrameTime = this->t;
                    break;
                case GLEQ_BUTTON_PRESSED:
                    this->onClick(event.mouse.button, true);
                    break;
                case GLEQ_BUTTON_RELEASED:
                    this->onClick(event.mouse.button, false);
                    break;
                case GLEQ_SCROLLED: 
                    this->camera.universalZoom(-event.scroll.y * 0.1f);
                    break;
                default:
                    break;
            }

            gleqFreeEvent(&event);
        }
        glfwPollEvents();
        this->mouseDeltaPos -= this->mousePos;

        // Wait until next frame
        this->t = glfwGetTime();
        const float dt = this->t - this->lastFrameTime;
        this->lastFrameTime = glfwGetTime();

        // Draw the scene
        this->simulate(dt);
        this->draw(dt);
        this->composeUI();
        this->idle();
        glfwSwapBuffers(this->window);
    }
}

void App::idle() {
}

void App::simulate(float dt) {
    const auto F = [&](const Vector3f& v) -> Vector3f {
        return v.cross(Vector3f(0.0f, 1.0f, 0.0f)) * v.norm() * 0.1f
            + Vector3f(sinf(v.y() + this->t), cosf(v.x() + this->t), cosf(this->t * 2.0f + v.x())) * 0.2f;
    };

    // Camera controls
    const float maxWinDim = (float)std::max(windowSize.x(), windowSize.y());
    const Vector2f panDelta = (this->mouseClickStart - this->mousePos) / maxWinDim;
    const Vector2f keyboardDelta = {
        (float)this->pressedKeys.count(GLFW_KEY_D) - (float)this->pressedKeys.count(GLFW_KEY_A),
        (float)this->pressedKeys.count(GLFW_KEY_W) - (float)this->pressedKeys.count(GLFW_KEY_S)
    };
    Vector2f dragDelta = Vector2f::Zero();
    if (this->mouseLeft) {
        dragDelta = panDelta * 2.0f;
        dragDelta.y() *= -1.0f;
    }
    if (this->pressedKeys.count(GLFW_KEY_LEFT_ALT)) {
        this->secondaryCamera.control(-this->mouseDeltaPos * dt * 0.15f, dragDelta, keyboardDelta * dt * 20.0f);
    } else {
        this->camera.control(-this->mouseDeltaPos * dt * 0.15f, dragDelta, keyboardDelta * dt * 20.0f);
    }

    // Physics simulation
    constexpr float dampingFactor = 0.25f;
    for (auto e : this->reg.view<PhysicsBody>()) {
        PhysicsBody& body = this->reg.get<PhysicsBody>(e);

        this->box.collide(body);

        body.acc = F(body.pos);
        body.vel += body.acc * dt;
        body.pos += body.vel * dt;
        body.vel *= 1.0f - (dampingFactor);
    }

    // Update model transforms
    for (auto e : this->reg.view<PhysicsBody, Model>()) {
        auto [body, model] = this->reg.get<PhysicsBody, Model>(e);
        model.pos = body.pos;
    }
    for (auto e : this->reg.view<Model, ModelTransform>()) {
        auto [model, transform] = this->reg.get<Model, ModelTransform>(e);
        transform.transform = model.transform();
    }

    // Update debug rays
    for (auto e : this->reg.view<PhysicsBody, DebugRay, RayTransform>()) {
        auto [body, ray, t] = this->reg.get<PhysicsBody, DebugRay, RayTransform>(e);
        ray.pos = body.pos;
        ray.length = body.vel.norm();
        ray.rot = {0.0f, 0.0f, angle2D(vec2(body.vel))};
    }
    for (auto e : this->reg.view<DebugRay, RayTransform>()) {
        auto [ray, t] = this->reg.get<DebugRay, RayTransform>(e);
        t.transform = ray.transform();
    }

    // Update lights
    for (auto e : this->reg.view<Light, uLight>()) {
        auto [light, ulight] = this->reg.get<Light, uLight>(e);
        ulight = light.toStruct(this->camera);
    }
}

void App::drawSky(const Matrix4f& view, const Matrix4f& proj) {
    this->skyProg.Bind();
    glBindVertexArray(this->vaoSky); $gl_err();

    this->skyProg.SetUniformMatrix4("uTInvViewProj", Transform3f(proj * view).inverse().matrix().data());
    this->skyTextures.bind(this->skyProg, "sky", "uSkyTex");

    glBindBuffer(GL_ARRAY_BUFFER, this->vboSky); $gl_err();

    glDepthMask(GL_FALSE);
    glDrawArrays(GL_TRIANGLES, 0, 6); $gl_err();
    glDepthMask(GL_TRUE);

    glBindBuffer(GL_ARRAY_BUFFER, 0); $gl_err();
    glBindTexture(GL_TEXTURE_2D, 0); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0); $gl_err();
}

void App::drawMeshes(const Matrix4f& view, const Matrix4f& proj, const Vector3f& camPos) {
    this->meshProg.Bind();
    glBindVertexArray(this->vaoMeshes); $gl_err();
    
    // Draw models in scene
    this->meshes.textures.bind(this->meshProg, "sky_cubemap", "uEnvTex");
    this->meshes.bind(this->meshProg);
    this->meshProg.SetUniformMatrix4("uTProj", proj.data());
    this->meshProg.SetUniformMatrix4("uTView", view.data());
    this->meshProg.SetUniform("uCamPos", (float)camPos.x(), (float)camPos.y(), (float)camPos.z());
    glMultiDrawElements(
        GL_TRIANGLES,
        this->vCounts.data(),
        GL_UNSIGNED_INT,
        (const void**)this->vOffsets.data(),
        this->vCounts.size()); $gl_err();

    glBindBuffer(GL_ARRAY_BUFFER, 0); $gl_err();
    glBindTexture(GL_TEXTURE_2D, 0); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0); $gl_err();
}

void App::drawDebug(const Matrix4f& view, const Matrix4f& proj) {
    this->wiresProg.Bind();
    glBindVertexArray(this->vaoWires); $gl_err();
    this->wiresProg.SetUniformMatrix4("uTProj", proj.data());
    this->wiresProg.SetUniformMatrix4("uTView", view.data());
    glBindBuffer(GL_ARRAY_BUFFER, this->vboWires); $gl_err();
    
    GLint attrib_vPos = this->wiresProg.AttribLocation("vPos"); $gl_err();
    glEnableVertexAttribArray(attrib_vPos); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0u, (void*)0u); $gl_err();

    auto& arrowsStorage = this->reg.view<RayTransform>().storage<RayTransform>();
    if (arrowsStorage.size() > 0) {
        glDrawArraysInstanced(GL_LINES, 0, 6, arrowsStorage.size()); $gl_err();
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0); $gl_err();
}

void App::updateBuffers() {
    // Meshes shader program
    this->meshProg.Bind();
    glBindVertexArray(this->vaoMeshes); $gl_err();
    this->meshProg.SetUniform("uViewport", (float)this->windowSize.x(), (float)this->windowSize.y());
    // Update lights buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights); $gl_err();
    auto& lightsStorage = this->reg.view<uLight>().storage<uLight>();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        lightsStorage.size() * sizeof(uLight), *lightsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->ssboLights); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
    this->meshProg.SetUniform("nLights", (GLuint)lightsStorage.size());
    // Update model transforms buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels); $gl_err();
    auto& transformsStorage = this->reg.view<ModelTransform>().storage<ModelTransform>();
    glBufferSubData(GL_SHADER_STORAGE_BUFFER,
        0, transformsStorage.size() * sizeof(Matrix4f), *transformsStorage.raw()); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
    
    // Wires shader program
    this->wiresProg.Bind();
    glBindVertexArray(this->vaoWires); $gl_err();
    // Update arrows buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrows); $gl_err();
    auto& arrowsStorage = this->reg.view<RayTransform>().storage<RayTransform>();
    if (arrowsStorage.size() > 0) {
        glBufferData(GL_SHADER_STORAGE_BUFFER,
            arrowsStorage.size() * sizeof(RayTransform), *arrowsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboArrows); $gl_err();
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
    // Update arrow colors buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrowColors); $gl_err();
    auto& colorsStorage = this->reg.view<DebugColor>().storage<DebugColor>();
    if (colorsStorage.size() > 0) {
        glBufferData(GL_SHADER_STORAGE_BUFFER,
            colorsStorage.size() * sizeof(DebugColor), *colorsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboArrowColors); $gl_err();
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
}

void App::draw(float dt) {
    this->updateBuffers();

    {
        // Reflect camera by plane (assumes plane is at origin and not rotated)
        const Vector3f reflPos(this->camera.pos.x(), -this->camera.pos.y(), this->camera.pos.z());
        const Matrix4f view = identityTransform()
            .rotate(euler({-this->camera.rot.x(), this->camera.rot.y(), 0.0f}))
            .translate(-reflPos)
            .matrix();
        const Matrix4f proj = this->camera.getProj(this->windowSize.cast<float>());

        this->hidden(this->ePlane, true);
        glBindFramebuffer(GL_FRAMEBUFFER, this->fboReflections); $gl_err();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); $gl_err();
        this->drawSky(view, proj);
        this->drawMeshes(view, proj, reflPos);
    }
    {
        this->hidden(this->ePlane, false);
        const Matrix4f view = this->camera.getView();
        const Matrix4f proj = this->camera.getProj(this->windowSize.cast<float>());
        glBindFramebuffer(GL_FRAMEBUFFER, 0); $gl_err();
        const GLuint targetTexID = this->meshes.getTextureData(this->meshes.getMaterial("plane_reflection").flatReflectionTexID).bindID;
        glGenerateTextureMipmap(this->texReflections); $gl_err();
        glGenerateTextureMipmap(targetTexID); $gl_err();
        glCopyImageSubData(
            this->texReflections, GL_TEXTURE_2D, 0, 0, 0, 0,
            targetTexID, GL_TEXTURE_2D, 0, 0, 0, 0,
            this->windowSize.x(), this->windowSize.y(), 1); $gl_err();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); $gl_err();
        this->drawSky(view, proj);
        this->drawMeshes(view, proj, this->camera.pos);

        this->drawDebug(view, proj);
    }
}

void App::composeUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(300, this->windowSize.y()));
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Text("Left Click: Pan");
    ImGui::Text("Wheel / Right Click: Zoom");
    ImGui::Text("1: Orbit camera");
    ImGui::Text("2: Fly camera");
    ImGui::Text("F6: Reload shaders");
    ImGui::Text(fmt::format("camera pos: {}", this->camera.pos).c_str());
    ImGui::Text(fmt::format("camera rot: {}", this->camera.rot).c_str());

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void App::hidden(entt::entity e, bool hidden) {
    auto [meshRef, objRef] = this->reg.get<MeshRef, ObjRef>(e);

    if (hidden) {
        this->vCounts.at(objRef.objID) = 0;
    } else {
        this->vCounts.at(objRef.objID) = meshRef.elemCount;
    }
}

entt::entity App::makeParticle() {
    entt::entity e = this->makeModel("sphere");

    this->reg.emplace<PhysicsBody>(e);
    this->reg.emplace<DebugRay>(e);
    this->reg.emplace<RayTransform>(e);
    DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
    debugColor.color = {0.5f, 0.5f, 0.5f, 1.0f};

    return e;
}

entt::entity App::makeModel(const std::string& name) {
    entt::entity e = this->reg.create();

    const MeshData& meshData = this->meshes.get(name);

    Model& model = this->reg.emplace<Model>(e);
    MeshRef& meshRef = this->reg.emplace<MeshRef>(e, meshData.ref);
    model.pivot = meshData.center;

    ObjRef& objRef = this->reg.emplace<ObjRef>(e);
    objRef.objID = this->vCounts.size();
    this->vCounts.push_back(meshRef.elemCount);
    this->vOffsets.push_back(meshRef.elemOffset);
    
    this->reg.emplace<ModelTransform>(e);
    return e;
}

entt::entity App::makeLight(const Vector3f& pos, const Vector3f& color, float intensity, LightType type) {
    entt::entity e = this->reg.create();

    Light& light = this->reg.emplace<Light>(e);
    light.pos = pos;
    light.color = color;
    light.intensity = intensity;
    light.type = type;

    this->reg.emplace<uLight>(e);

    // DebugRay& debugRay = this->reg.emplace<DebugRay>(e);
    // debugRay.pos = pos;
    // debugRay.rot = towards(pos, {0.0f, 0.0f, 0.0f});

    // RayTransform& rayTransform = this->reg.emplace<RayTransform>(e);
    // rayTransform.transform = debugRay.transform();

    // DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
    // debugColor.color = {1.0f, 1.0f, 0.2f, 1.0f};

    return e;
}
