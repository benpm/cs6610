
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

App::App() {
    // Initialize GLFW and Gleq
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    this->window = glfwCreateWindow(
        this->windowSize.x(),
        this->windowSize.y(), "CS6610", NULL, NULL);
    if (!this->window) {
        spdlog::error("Could not open GLFW window");
        exit(-1);
    }
    gleqTrackWindow(this->window);
    glfwMakeContextCurrent(this->window);

    // Init glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        spdlog::error("Failed to initialize GLAD");
    }

    glfwSwapInterval(1);

    // Initialize imgui
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(this->window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // OpenGL config
    // glEnable(GL_PROGRAM_POINT_SIZE);
    // glEnable(GL_POINT_SPRITE);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_MULTISAMPLE);
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);
    glLineWidth(2.0f);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Build and bind meshes shader program
    bool built = this->wiresProg.BuildFiles(
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

    glGenBuffers(1, &this->ssboArrows);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrows);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboArrows);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glGenBuffers(1, &this->ssboArrowColors);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrowColors);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboArrowColors);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    
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
    glGenVertexArrays(1, &this->vaoMeshes);
    glBindVertexArray(this->vaoMeshes);
    
    // Create models
    meshes.add("resources/models/teapot.obj", "", true);
    meshes.add("resources/models/suzanne.obj", "", true);
    meshes.add("resources/models/sphere.obj", "", true);
    meshes.add("resources/models/cube.obj", "", true);
    meshes.build(this->meshProg);
    spdlog::info("Loaded meshes");

    for (size_t i = 0; i < this->objectsToGen; i++) {
        entt::entity e = this->reg.create();

        Model& model = this->reg.emplace<Model>(e);
        model.scale = Vector3f::Ones() * rng.range(1.0f, 5.0f);
        model.pos = rng.vec(this->box);
        model.rot = rng.rotation();

        MeshData& meshData = this->reg.emplace<MeshData>(e, meshes.get(rng.choose({"teapot", "suzanne", "sphere"})));
        model.pivot = meshData.center;
        this->vCounts.push_back(meshData.elemCount);
        this->vOffsets.push_back(meshData.elemOffset);
        
        uMaterial& mat = this->reg.emplace<uMaterial>(e);
        float hue = rng.range(0.0f, 360.0f);
        mat.diffuseColor = hsvToRgb({hue, 0.8f, 0.7f});
        mat.specularColor = hsvToRgb({hue, 0.4f, 1.0f});
        mat.shininess = 2000.0f;

        ModelTransform& transform = this->reg.emplace<ModelTransform>(e);
        transform.transform = model.transform();
    }
    spdlog::debug("placed {} objects", this->objectsToGen);

    this->camera.orbitDist(50.0f);

    // Add lights
    this->sunlight = this->lights.emplace_back(std::make_shared<Light>(
        Vector3f(0.25f, 0.5f, 0.25f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.9f,
        LightType::directional));
    this->lights.emplace_back(std::make_shared<Light>(
        Vector3f(-0.15f, -0.5f, -0.45f),
        Vector3f(1.0f, 1.0f, 0.9f),
        0.1f,
        LightType::directional));

    // Create and bind model transforms SSBO
    glGenBuffers(1, &this->ssboModels);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels);
    auto& transformsStorage = this->reg.view<ModelTransform>().storage<ModelTransform>();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        transformsStorage.size() * sizeof(Matrix4f),
        *transformsStorage.raw(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    $gl_err();

    // Create and bind materials SSBO
    glGenBuffers(1, &this->ssboMaterials);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials);
    auto& materialsStorage = this->reg.view<uMaterial>().storage<uMaterial>();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        materialsStorage.size() * sizeof(Matrix4f),
        *materialsStorage.raw(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    $gl_err();

    // Create and bind lights SSBO
    glGenBuffers(1, &this->ssboLights);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights);
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->lights.size() * sizeof(uLight), NULL, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->ssboLights);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    $gl_err();
    this->meshProg.SetUniform("nLights", (uint32_t)this->lights.size());
}

App::~App() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();
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
                    spdlog::error("Failed to build shader program");
                } else {
                    this->meshProg.Bind();
                    spdlog::info("Rebuilt shader program");
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
        this->mouseDeltaPos = {0.0f, 0.0f};
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
                    this->mouseDeltaPos = Vector2f(event.pos.x, event.pos.y) - this->mousePos;
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

        // Wait until next frame
        this->t = glfwGetTime();
        const float dt = this->t - this->lastFrameTime;
        this->lastFrameTime = glfwGetTime();

        // Draw the scene
        glClearColor(0.08f, 0.08f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        this->simulate(dt);
        this->draw(dt);
        this->idle();
        glfwSwapBuffers(this->window);
    }
}

void App::idle() {
}

void App::simulate(float dt) {
    constexpr auto F = [](const Vector3f& v) -> Vector3f {
        const float a = angle2D(vec2(v));
        const float d = v.norm();
        return {cosf(a + tau4) * d, sinf(a + tau4) * d, 0.0f};
    };

    // Physics simulation
    for (auto e : this->reg.view<PhysicsBody>()) {
        PhysicsBody& body = this->reg.get<PhysicsBody>(e);

        if (body.pos.x() < this->box.min.x()) {
            body.pos.x() = this->box.min.x();
            body.vel.x() = -body.vel.x();
        } else if (body.pos.x() > this->box.max.x()) {
            body.pos.x() = this->box.max.x();
            body.vel.x() = -body.vel.x();
        }
        if (body.pos.y() < this->box.min.y()) {
            body.pos.y() = this->box.min.y();
            body.vel.y() = -body.vel.y();
        } else if (body.pos.y() > this->box.max.y()) {
            body.pos.y() = this->box.max.y();
            body.vel.y() = -body.vel.y();
        }

        switch (this->simulationMethod) {
            case SimulationMethod::explicitEuler: {
                const float dampingFactor = 2.0f;
                body.acc = F(body.pos);
                body.vel += body.acc * dt;
                body.pos += body.vel * dt;
                body.vel *= 1.0f - (dampingFactor * dt);
            } break;
            case SimulationMethod::implicitEuler: {
                const size_t iters = 100;
                const float step = 0.25f;
                const float s = dt * (step / (float)iters);
                for (size_t i = 0; i < iters; i++) {
                    const Vector3f x = body.pos + body.vel*s + F(body.pos)*s*s;
                    body.acc = (x - body.pos) / (dt * dt) - body.vel / dt;
                    body.vel += body.acc * s;
                    body.pos += body.vel * s;
                }
            } break;
            case SimulationMethod::verlet: {
                body.acc = F(body.pos);
                body.vel += body.acc * dt * 0.5f;
                body.pos += body.vel * dt;
            } break;
            case SimulationMethod::implicit: {
                const size_t iters = 10;
                const float step = 0.01f;
                const float s = step / (float)iters;
                for (size_t i = 0; i < iters; i++) {
                    const float d = body.pos.norm();
                    const float a = angle2D(vec2(body.pos)) + tau4 + 1.0e-8f;
                    const Vector3f np = {cosf(a) * d, sinf(a) * d, 0.0f};
                    const Vector3f dp = np - body.pos;
                    body.acc = dp / (dt * dt) - body.vel / dt;
                    body.vel += body.acc * dt * s;
                    body.pos += body.vel * dt * s;
                }
            } break;
            case SimulationMethod::velocityOnly: {
                const float a = angle2D(vec2(body.pos));
                const float d = body.pos.norm();
                body.vel = {cosf(a + tau4 - 0.1f) * d, sinf(a + tau4 - 0.1f) * d, 0.0f};
                body.pos += body.vel * dt;
            } break;
        }
    }

    // Update model transforms
    for (auto e : this->reg.view<PhysicsBody, Model, ModelTransform>()) {
        auto [body, model, transform] = this->reg.get<PhysicsBody, Model, ModelTransform>(e);
        model.pos = body.pos;
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
}

void App::draw(float dt) {
    // Camera controls
    const float maxWinDim = (float)std::max(windowSize.x(), windowSize.y());
    const Vector2f panDelta = (this->mouseClickStart - this->mousePos) / maxWinDim * tau2;
    const Vector2f keyboardDelta = {
        (float)this->pressedKeys.count(GLFW_KEY_D) - (float)this->pressedKeys.count(GLFW_KEY_A),
        (float)this->pressedKeys.count(GLFW_KEY_W) - (float)this->pressedKeys.count(GLFW_KEY_S)
    };
    Vector2f dragDelta = Vector2f::Zero();
    if (this->mouseLeft) {
        dragDelta = panDelta * dt * 20.0f;
        dragDelta.y() *= -1.0f;
    }
    if (this->mouseMoved) {
        this->camera.control(this->mouseDeltaPos * dt, dragDelta, keyboardDelta * dt * 20.0f);
    }

    this->meshProg.Bind();
    glBindVertexArray(this->vaoMeshes); $gl_err();

    // Set up transformation matrices
    const Matrix4f tProj = this->camera.getProj(this->windowSize.cast<float>());
    this->meshProg.SetUniformMatrix4("uTProj", tProj.data());
    const Matrix4f tView = this->camera.getView();
    this->meshProg.SetUniformMatrix4("uTView", tView.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    // Update lights
    std::vector<uLight> lightStructs(this->lights.size());
    for (size_t i = 0; i < this->lights.size(); i++) {
        lightStructs[i] = this->lights[i]->toStruct(this->camera);
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboLights); $gl_err();
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, lightStructs.size() * sizeof(uLight), lightStructs.data()); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->ssboLights); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboModels); $gl_err();
    auto& transformsStorage = this->reg.view<ModelTransform>().storage<ModelTransform>(); $gl_err();
    glBufferSubData(GL_SHADER_STORAGE_BUFFER,
        0, transformsStorage.size() * sizeof(Matrix4f), *transformsStorage.raw()); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboModels); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
    
    // Draw models in scene
    this->meshes.bind();
    glMultiDrawElements(
        GL_TRIANGLES,
        this->vCounts.data(),
        GL_UNSIGNED_INT,
        (const void**)this->vOffsets.data(),
        this->vCounts.size()); $gl_err();
    
    // Draw wireframe stuff
    this->wiresProg.Bind();
    glBindVertexArray(this->vaoWires); $gl_err();
    this->wiresProg.SetUniformMatrix4("uTProj", tProj.data());
    this->wiresProg.SetUniformMatrix4("uTView", tView.data());
    glBindBuffer(GL_ARRAY_BUFFER, this->vboWires); $gl_err();
    
    GLuint attrib_vPos = this->wiresProg.AttribLocation("vPos"); $gl_err();
    glEnableVertexAttribArray(attrib_vPos); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0u, (void*)0u); $gl_err();

    std::vector<RayTransform> rayTransforms = {{.transform = identityTransform().matrix()}};
    std::vector<DebugColor> debugColors = {{.color = {1.0f, 1.0f, 1.0f, 1.0f}}};
    for (auto e : this->reg.view<RayTransform, DebugColor>()) {
        auto [rayT, color] = this->reg.get<RayTransform, DebugColor>(e);
        rayTransforms.push_back(rayT);
        debugColors.push_back(color);
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrows); $gl_err();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        rayTransforms.size() * sizeof(RayTransform), rayTransforms.data(), GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->ssboArrows); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrowColors); $gl_err();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        debugColors.size() * sizeof(DebugColor), debugColors.data(), GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboArrowColors); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    glDrawArraysInstanced(GL_LINES, 0, 6, rayTransforms.size()); $gl_err();

    // Draw path
    glBindBuffer(GL_ARRAY_BUFFER, this->vboPath); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0u, (void*)0u); $gl_err();
    glBufferData(GL_ARRAY_BUFFER,
        this->particlePath.size() * sizeof(Vector3f), this->particlePath.data(), GL_DYNAMIC_DRAW); $gl_err();
    glDrawArrays(GL_LINE_STRIP, 0, this->particlePath.size()); $gl_err();

    // Draw box
    const auto corners = this->box.corners();
    glBindBuffer(GL_ARRAY_BUFFER, this->vboBox); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0u, (void*)0u); $gl_err();
    glBufferData(GL_ARRAY_BUFFER,
        corners.size() * sizeof(Vector3f), corners.data(), GL_DYNAMIC_DRAW); $gl_err();
    glDrawArrays(GL_LINE_LOOP, 0, corners.size()); $gl_err();


    this->composeUI();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void App::composeUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(200, 500));
    ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::TextWrapped(
        "Particle simulation! Use left mouse to add a force, right mouse to pan camera."
    );
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    // Dropdown to choose simulation method
    if (ImGui::BeginCombo("Simulation Method", this->simMethodNames.at(this->simulationMethod).c_str())) {
        for (int i = 0; i < this->simMethodNames.size(); i++) {
            const SimulationMethod s = (SimulationMethod)i;
            bool isSelected = (this->simulationMethod == s);
            if (ImGui::Selectable(this->simMethodNames.at(s).c_str(), isSelected)) {
                this->simulationMethod = s;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    auto bodies = this->reg.view<PhysicsBody>();

    // Box size slider
    float boxSize = this->box.width();
    ImGui::SliderFloat("Box Size", &boxSize, 1.0f, 100.0f);
    this->box.size(boxSize);

    // Button that sets all velocities to zero
    if (ImGui::Button("Reset Velocities")) {
        for (auto e : bodies) {
            PhysicsBody& body = this->reg.get<PhysicsBody>(e);
            body.vel = Vector3f::Zero();
        }
    }

    // Particle count slider
    int particleCount = bodies.size();
    ImGui::SliderInt("Particle Count", &particleCount, 1, 500);
    while (particleCount != bodies.size()) {
        if (particleCount < bodies.size()) {
            this->reg.destroy(*bodies.begin());
        } else {
            this->makeParticle();
        }
    }

    ImGui::End();
}

entt::entity App::makeParticle() {
    entt::entity e = this->reg.create();

    Model& model = this->reg.emplace<Model>(e);
    model.pos = this->rng.vec(this->box);
    model.pos.z() = 0.0f;
    model.scale *= 0.5f;

    MeshData& meshData = this->reg.emplace<MeshData>(e, this->meshes.get("sphere"));
    this->vCounts.push_back(meshData.elemCount);
    this->vOffsets.push_back(meshData.elemOffset);
    model.pivot = meshData.center;
    
    uMaterial& mat = this->reg.emplace<uMaterial>(e);
    mat.ambientColor = {0.08f, 0.08f, 0.08f};

    ModelTransform& transform = this->reg.emplace<ModelTransform>(e);
    transform.transform = model.transform();

    PhysicsBody& body = this->reg.emplace<PhysicsBody>(e);
    body.pos = model.pos;

    DebugRay& ray = this->reg.emplace<DebugRay>(e);
    ray.pos = model.pos;
    ray.rot = {tau4, 0.0f, tau4};
    ray.length = 1.0f;

    DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
    debugColor.color = {0.5f, 0.5f, 0.5f, 1.0f};

    RayTransform& rayTransform = this->reg.emplace<RayTransform>(e);
    rayTransform.transform = ray.transform();

    return e;
}