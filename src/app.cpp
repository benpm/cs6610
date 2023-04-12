
#ifdef PLATFORM_WINDOWS
    #define NOMINMAX
#endif

#include <cassert>
#include <functional>
#include <algorithm>
#include <app.hpp>
#define GLEQ_IMPLEMENTATION
#include <gleq.h>
#include <spdlog/formatter.h>
#include <physics.hpp>
#include <gfx.hpp>

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

    int dpiScale = 2;
    float monScaleX, monScaleY;
    glfwGetMonitorContentScale(glfwGetPrimaryMonitor(), &monScaleX, &monScaleY);
    dpiScale = std::max((int)monScaleX, (int)monScaleY);
    ImGui::GetStyle().ScaleAllSizes((float)dpiScale);
    ImGui::GetIO().FontGlobalScale = (float)dpiScale;
    ImFontConfig fontConfig;
    fontConfig.OversampleH = 2;
    fontConfig.OversampleV = 2;
    fontConfig.SizePixels = 16.0f * dpiScale;
    this->ui.font = ImGui::GetIO().Fonts->AddFontFromFileTTF(
        "resources/fonts/RobotoMono-Medium.ttf", 16.0f, &fontConfig);

    this->loadingScreen("building shader programs, creating buffers");

    // OpenGL config
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_MULTISAMPLE);
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);
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
    // Create arrow that points in the -z direction
    std::vector<Vector3f> arrowVerts = {
        {+0.0f, +0.0f, +0.0f},
        {+0.0f, +0.0f, -1.0f},

        {+0.0f, +0.0f, -1.0f},
        {-0.1f, +0.0f, -0.9f},

        {+0.0f, +0.0f, -1.0f},
        {+0.1f, +0.0f, -0.9f},
    };
    glBufferData(GL_ARRAY_BUFFER, arrowVerts.size() * sizeof(Vector3f),
        arrowVerts.data(), GL_STATIC_DRAW); $gl_err();

    glCreateBuffers(1, &this->vboPath); $gl_err();
    glCreateBuffers(1, &this->vboBox); $gl_err();
    glCreateBuffers(1, &this->ssboArrows); $gl_err();
    glCreateBuffers(1, &this->ssboArrowColors); $gl_err();

    // Build and bind shadows shader program
    built = this->depthProg.BuildFiles(
        "resources/shaders/shadow.vert",
        "resources/shaders/shadow.frag");
    if (!built) {
        spdlog::error("Failed to build shadows shader program");
    } else {
        this->depthProg.Bind();
    }
    
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
    this->meshes.add("resources/models/cube.obj", "", true, false);
    this->meshes.add("resources/models/suzanne.obj");
    this->meshes.add("resources/models/sphere.obj");
    size_t skyMatID = this->meshes.createSkyMaterial("resources/textures/cubemap");
    // this->meshes.setMaterial("teapot", skyMatID);
    // this->meshes.setMaterial("sphere", skyMatID);
    size_t whiteMatID = this->meshes.createMaterial("white", uMaterial{
        .diffuseColor = {0.8f, 0.8f, 0.8f},
        .shininess = 500.0f
    });
    size_t lightMatID = this->meshes.createMaterial("light", uMaterial{
        .emissionColor = {1.0f, 1.0f, 1.0f},
        .emissionFactor = 1.0f
    });
    this->meshes.setMaterial("suzanne", whiteMatID);
    this->meshes.clone("sphere", "light_ball");
    this->meshes.setMaterial("light_ball", lightMatID);

    uMaterial planeRefl {
        .diffuseColor = Vector3f::Zero(),
        .shininess = 1000.0f,
        .flatReflectionTexID = (int)this->texReflections,
    };
    this->meshes.createMaterial("plane_reflection", planeRefl);
    // this->meshes.setMaterial("quad", "plane_reflection");

    uMaterial& skyRefl = this->meshes.getMaterial(skyMatID);
    skyRefl.shininess = 1000.0f;
    skyRefl.diffuseColor = Vector3f::Zero();
    
    glGenFramebuffers(1, &this->fboShadows); $gl_err();

    spdlog::info("Loaded meshes");

    int reflectionLayer = 0;

    // Construct scene
    this->loadingScreen("constructing scene");
    for (size_t i = 0; i < 1; i++) {
        int reflLayer = reflectionLayer++;

        entt::entity e = this->makeModel(this->meshes.clone("sphere", uMaterial {
            .reflectionLayer = reflLayer
        }));
        Model& model = this->reg.get<Model>(e);
        model.scale = Vector3f::Ones() * rng.range(0.2f, 2.0f);
        model.pos = rng.vec(this->box);

        ReflectionProbe& reflProbe = this->reg.emplace<ReflectionProbe>(e);
        reflProbe.layer = reflLayer;
    }

    {
        entt::entity e = this->reg.create();
        SpringMesh s ("resources/vmodels/cube.ele", "resources/vmodels/cube.node");
        SpringMesh &springMesh = this->reg.emplace<SpringMesh>(e,s);

        const MeshData& meshData = this->meshes.add("dragon", springMesh.surfaceVertices, springMesh.surfaceElems);
        uMaterial& material = this->meshes.getMaterial("dragon.mat");
        material.ambientColor = {1.0f, 1.0f, 1.0f};
        material.ambientFactor = 0.15f;

        Model& model = this->reg.emplace<Model>(e, meshData);
        model.pivot = Vector3f::Zero();
        MeshRef& meshRef = this->reg.emplace<MeshRef>(e, meshData.ref);

        this->reg.emplace<ObjRef>(e, this->makeObj(meshRef));
        
        this->reg.emplace<ModelTransform>(e);

        this->eSpringMesh = e;
    }
    { // Create a point to visualize mouse select
        entt::entity e = this->makeModel(this->meshes.clone("sphere", uMaterial {
                .emissionColor = {1.0f, 1.0f, 0.0f},
                .emissionFactor = 1.0f
            }));
        this->eSelectPoint = e;

        Model& model = this->reg.get<Model>(e);
        model.scale = Vector3f::Ones() * 0.2f;
    }
    { // Create an arrow to visualize added forces
        entt::entity e = this->reg.create();

        this->reg.emplace<DebugRay>(e);
        this->reg.emplace<RayTransform>(e);
        DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
        debugColor.color = {1.0f, 1.0f, 0.0f, 0.0f};

        this->eDragArrow = e;
    }
    // Create lots of colorful boxes
    for (size_t i = 0; i < 0; i++) {
        entt::entity e = this->makeRigidBody(
            this->meshes.clone("cube", uMaterial {
                .diffuseColor = hsvToRgb({rng.range(0.0f, 360.0f), 1.0f, 1.0f}),
                .shininess = 1000.0f
            }),
            rng.vec({1.6f, 0.2f, 1.6f}, {1.8f, 0.4f, 1.8f}),
            rng.vec(this->box));
        RigidBody& body = this->reg.get<RigidBody>(e);
        body.angMomentum = rng.vec({-0.02f, -0.02f, -0.02f}, {0.02f, 0.02f, 0.02f});
        PhysicsBody& pbody = this->reg.get<PhysicsBody>(e);
        pbody.vel = rng.vec({-0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, 0.5f});
    }

    // Create debug axis arrows
    for (size_t i = 0; i < 3; i++) {
        entt::entity e = this->reg.create();
        DebugRay& debugRay = this->reg.emplace<DebugRay>(e);
        debugRay.rot = dirToRot(Vector3f::Unit(i));
        debugRay.pos = {0.0f, 2.5f, 0.0f};
        debugRay.length = 1.0f;
        RayTransform& rayTransform = this->reg.emplace<RayTransform>(e);
        rayTransform.transform = debugRay.transform();
        DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
        debugColor.color = {0.0f, 0.0f, 0.0f, 1.0f};
        debugColor.color[i] = 1.0f;
    }

    spdlog::debug("placed {} objects", this->reg.view<Model>().size());

    this->cameraControl.orbitDist(3.0f);

    // Add lights
    this->makeLight(
        Vector3f(2.0f, 3.0f, 1.0f),
        Vector3f(1.0f, 1.0f, 0.9f),
        8.0f, 3.0f,
        LightType::point);
    this->makeLight(
        Vector3f(-2.0f, 3.0f, -1.0f),
        Vector3f(0.1f, 0.2f, 0.9f),
        8.0f, 3.0f,
        LightType::point);
    {
        this->eSpotLight = this->makeSpotLight(
            Vector3f::Zero(),
            Vector3f::Zero(),
            Vector3f(0.5f, 1.0f, 0.6f),
            24.0f, 1.0f);
        this->lightControl.orbitDist(8.0f);
        this->lightControl.orbitTheta(tau4);
        this->lightControl.orbitPhi(0.0f);
        this->lightControl.update(this->reg.get<Light>(this->eSpotLight));
        this->lightControl.update(this->reg.get<Camera>(this->eSpotLight));
    }
    this->makeSpotLight(
        Vector3f(4.0f, 6.0f, 4.0f),
        Vector3f(-4.0f, -6.0f, -4.0f),
        Vector3f(1.0f, 0.2f, 0.6f),
        24.0f, 1.0f);

    // Shadows cube map
    int nPointLights = 0;
    int nSpotLights = 0;
    for (entt::entity e : this->reg.view<Light>()) {
        if (this->reg.get<Light>(e).type == LightType::point) {
            nPointLights++;
        } else if (this->reg.get<Light>(e).type == LightType::spot) {
            nSpotLights++;
        }
    }
    this->texCubeShadows = gfx::texture(gfx::TextureConfig{
        .target = GL_TEXTURE_CUBE_MAP_ARRAY,
        .format = GL_DEPTH_COMPONENT,
        .width = shadowMapSize,
        .height = shadowMapSize,
        .wrap = GL_CLAMP_TO_EDGE,
        .storageType = GL_FLOAT,
        .shadow = true,
        .filter = GL_NEAREST,
        .layers = nPointLights,
    });
    this->meshes.textures.add("shadow_map", this->texCubeShadows, GL_TEXTURE_CUBE_MAP_ARRAY, TextureSampler::shadow);

    // Spotlight shadow map
    this->texSpotShadows = gfx::texture(gfx::TextureConfig{
        .target = GL_TEXTURE_2D_ARRAY,
        .format = GL_DEPTH_COMPONENT,
        .width = shadowMapSize,
        .height = shadowMapSize,
        .wrap = GL_CLAMP_TO_BORDER,
        .storageType = GL_FLOAT,
        .shadow = true,
        .filter = GL_NEAREST,
        .layers = nSpotLights,
    });
    this->meshes.textures.add("spot_shadow_map", this->texSpotShadows, GL_TEXTURE_2D_ARRAY, TextureSampler::shadow);

    // Reflection probes cubemap array
    this->texReflections = gfx::texture(gfx::TextureConfig{
        .target = GL_TEXTURE_CUBE_MAP_ARRAY,
        .format = GL_RGB,
        .width = shadowMapSize,
        .height = shadowMapSize,
        .wrap = GL_CLAMP_TO_EDGE,
        .storageType = GL_UNSIGNED_BYTE,
        .mipmap = true,
        .filter = GL_LINEAR,
        .layers = reflectionLayer,
    });
    glGenFramebuffers(1, &this->fboReflections); $gl_err();
    glBindFramebuffer(GL_FRAMEBUFFER, this->fboReflections); $gl_err();
    glGenRenderbuffers(1, &this->rboReflections); $gl_err();
    glBindRenderbuffer(GL_RENDERBUFFER, this->rboReflections); $gl_err();
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, shadowMapSize, shadowMapSize); $gl_err();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, this->rboReflections); $gl_err();
    glBindRenderbuffer(GL_RENDERBUFFER, 0); $gl_err();
    glBindFramebuffer(GL_FRAMEBUFFER, 0); $gl_err();
    this->meshes.textures.add("refl_maps", this->texReflections, GL_TEXTURE_CUBE_MAP_ARRAY, TextureSampler::cubemap);

    // Setup render passes
    this->renderPasses.push_back(RenderPass{ // Cubemap shadow pass
        .type = RenderPass::Type::cubemap,
        .fbo = this->fboShadows,
        .viewport = {{(float)shadowMapSize, (float)shadowMapSize}},
        .arrayTarget = RenderTarget {
            .type = RenderTarget::Type::cubemap,
            .id = this->texCubeShadows,
            .attachment = GL_DEPTH_ATTACHMENT,
        }
    });
    this->renderPasses.push_back(RenderPass{ // Spotlight shadow pass
        .type = RenderPass::Type::array,
        .fbo = this->fboShadows,
        .viewport = {{(float)shadowMapSize, (float)shadowMapSize}},
        .objMask = {},
        .arrayTarget = RenderTarget {
            .type = RenderTarget::Type::textureArray,
            .id = this->texSpotShadows,
            .attachment = GL_DEPTH_ATTACHMENT,
        },
    });
    this->renderPasses.push_back(RenderPass{ // Reflection pass
        .type = RenderPass::Type::cubemap,
        .fbo = this->fboReflections,
        .viewport = {{(float)shadowMapSize, (float)shadowMapSize}},
        .targets = {
            RenderTarget {
                .type = RenderTarget::Type::renderbuffer,
                .id = this->rboReflections,
                .attachment = GL_DEPTH_ATTACHMENT,
            }
        },
        .arrayTarget = RenderTarget {
            .type = RenderTarget::Type::textureArray,
            .id = this->texReflections,
            .attachment = GL_COLOR_ATTACHMENT0,
        },
    });
    this->renderPasses.push_back(RenderPass{ // Final pass
        .type = RenderPass::Type::final,
        .fbo = GL_NONE
    });

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

    this->meshes.build(this->meshProg);
}

App::~App() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();
}

void App::loadingScreen(const std::string& message) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(this->windowSize.x(), this->windowSize.y()));
    ImGui::Begin("Loading...", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBackground
        | ImGuiWindowFlags_NoDecoration);
    auto textSize = ImGui::CalcTextSize(message.c_str());
    ImGui::SetCursorPos(ImVec2(
        this->windowSize.x() / 2.0f - textSize.x / 2.0f,
        this->windowSize.y() / 2.0f - textSize.y / 2.0f));
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
                const char* progShaderNames[] = {
                    "basic", "sky", "shadow"
                };
                const std::array<cyGLSLProgram*, 3> progs = {
                    &this->meshProg, &this->skyProg, &this->depthProg
                };
                for (size_t i = 0; i < progs.size(); ++i) {
                    const std::string vertPath = fmt::format("resources/shaders/{}.vert", progShaderNames[i]);
                    const std::string fragPath = fmt::format("resources/shaders/{}.frag", progShaderNames[i]);
                    cyGLSLShader vertShader;
                    cyGLSLShader fragShader;
                    bool success = false;
                    success |= vertShader.CompileFile(vertPath.c_str(), GL_VERTEX_SHADER);
                    success |= fragShader.CompileFile(fragPath.c_str(), GL_FRAGMENT_SHADER);
                    if (success) {
                        bool built = progs[i]->Build(&vertShader, &fragShader);
                        if (!built) {
                            spdlog::error("Failed to build {} shader program", progShaderNames[i]);
                        }
                        spdlog::info("Rebuilt {} shader program", progShaderNames[i]);
                    } else {
                        spdlog::warn("Failed to build {} shader program", progShaderNames[i]);
                    }
                }
            } break;
            case GLFW_KEY_1: {
                this->cameraControl.mode = CameraControl::Mode::orbit;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            } break;
            case GLFW_KEY_2: {
                this->cameraControl.mode = CameraControl::Mode::fly;
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
            this->cameraControl.dragStart();
            this->lightControl.dragStart();
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
        if (button == GLFW_MOUSE_BUTTON_RIGHT && this->eSelected != entt::null) {
            auto [debugRay, debugColor] = this->reg.get<DebugRay, DebugColor>(this->eDragArrow);
            debugRay.pos = this->selectPoint;
            debugColor.color.w() = 1.0f;
        }
    } else {
        if (button == GLFW_MOUSE_BUTTON_RIGHT && this->eSelected != entt::null) {
            auto [debugRay, debugColor] = this->reg.get<DebugRay, DebugColor>(this->eDragArrow);
            auto [rb, pb] = this->reg.get<RigidBody, PhysicsBody>(this->eSelected);
            rb.applyImpulse(pb, this->selectPoint, this->selectPoint + direction(debugRay.rot) * debugRay.length * 0.2f);
            debugColor.color.w() = 0.0f;
        }
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
                    this->cameraControl.universalZoom(-event.scroll.y * 0.1f);
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
    // Update select point
    const Ray camRay = this->camera.getRay(this->mousePos, this->windowSize.cast<float>());
    if (!this->mouseRight) {
        this->eSelected = entt::null;
        std::optional<Vector3f> nearestHit = std::nullopt;
        for (const auto& e : this->reg.view<RigidBody, PhysicsBody, ColliderBox>()) {
            const auto [rigidBody, physicsBody, colliderBox] = this->reg.get<RigidBody, PhysicsBody, ColliderBox>(e);
            std::optional<Vector3f> hit = rigidBody.intersect(physicsBody, colliderBox, camRay);
            if (hit && (!nearestHit || (camRay.origin - *hit).norm() < (camRay.origin - *nearestHit).norm())) {
                nearestHit = hit;
                this->eSelected = e;
            }
        }
        if (nearestHit) {
            this->hidden(this->eSelectPoint, false);
            selectPoint = *nearestHit;
            this->reg.get<Model>(this->eSelectPoint).pos = selectPoint;
        } else {
            this->hidden(this->eSelectPoint, true);
        }
    } else {
        this->reg.get<DebugRay>(this->eDragArrow).setEndpoint(
            camRay.origin + camRay.direction * (camRay.origin - this->selectPoint).norm());
    }

    // Camera control and update
    const float maxWinDim = (float)std::max(windowSize.x(), windowSize.y());
    const Vector2f panDelta = (this->mouseClickStart - this->mousePos) / maxWinDim;
    const Vector2f keyboardDelta = {
        (float)this->pressedKeys.count(GLFW_KEY_D) - (float)this->pressedKeys.count(GLFW_KEY_A),
        (float)this->pressedKeys.count(GLFW_KEY_W) - (float)this->pressedKeys.count(GLFW_KEY_S)
    };
    Vector2f dragDelta = Vector2f::Zero();
    if (this->mouseLeft && !this->mouseRight) {
        dragDelta = panDelta * 2.0f;
        dragDelta.y() *= -1.0f;
    }
    if (this->pressedKeys.count(GLFW_KEY_LEFT_SHIFT)) {
        this->lightControl.control(-this->mouseDeltaPos * dt * 0.15f, dragDelta, keyboardDelta * dt * 20.0f);
        this->lightControl.update(this->reg.get<Light>(this->eSpotLight));
    } else {
        this->cameraControl.control(-this->mouseDeltaPos * dt * 0.15f, dragDelta, keyboardDelta * dt * 20.0f);
    }
    this->cameraControl.update(this->camera);
    this->lightControl.update(this->reg.get<Light>(this->eSpotLight));

    // Simulate rigid body dynamics
    for (auto e : this->reg.view<PhysicsBody, RigidBody, ColliderBox>()) {
        auto [pb, rb, collider] = this->reg.get<PhysicsBody, RigidBody, ColliderBox>(e);
        const float substep = dt * (this->simTimeStep / (float)this->simTimeIters);
        for (size_t i = 0; i < this->simTimeIters; ++i) {
            this->box.collide(rb, pb, collider);
            Physics::simulate(substep, rb, pb);
        }
    }

    // Simulate spring meshes
    for (auto e : this->reg.view<SpringMesh>()) {
        SpringMesh& mesh = this->reg.get<SpringMesh>(e);
        this->box.collide(mesh);
        mesh.simulate(dt * this->simTimeStep);
        this->meshes.updateVertices("dragon", mesh.surfaceVertices);
    }

    // Update model transforms from various sources
    for (auto e : this->reg.view<RigidBody, Model>()) {
        auto [body, model] = this->reg.get<RigidBody, Model>(e);
        model.rot = body.rot.eulerAngles(0, 1, 2);
    }
    for (auto e : this->reg.view<PhysicsBody, Model>()) {
        auto [body, model] = this->reg.get<PhysicsBody, Model>(e);
        model.pos = body.pos;
    }
    for (auto e : this->reg.view<Light, Model>()) {
        auto [light, model] = this->reg.get<Light, Model>(e);
        model.pos = light.pos;
    }
    for (auto e : this->reg.view<Model, ModelTransform>()) {
        auto [model, transform] = this->reg.get<Model, ModelTransform>(e);
        transform.transform = model.transform();
    }

    // Update debug stuff
    for (auto e : this->reg.view<PhysicsBody, DebugRay, RayTransform>()) {
        auto [body, ray, t] = this->reg.get<PhysicsBody, DebugRay, RayTransform>(e);
        ray.pos = body.pos;
        ray.length = body.vel.norm();
        ray.rot = {0.0f, 0.0f, angle2D(vec2(body.vel))};
    }
    for (auto e : this->reg.view<Light, DebugRay>()) {
        auto [light, ray] = this->reg.get<Light, DebugRay>(e);
        ray.pos = light.pos;
        ray.rot = dirToRot(light.dir);
    }
    for (auto e : this->reg.view<DebugRay, RayTransform>()) {
        auto [ray, t] = this->reg.get<DebugRay, RayTransform>(e);
        t.transform = ray.transform();
    }

    // Update lights
    for (auto e : this->reg.view<Light, Camera>()) {
        auto [light, cam] = this->reg.get<Light, Camera>(e);
        light.shadowCam(cam);
    }
    std::array<uint32_t, 2u> shadowMapLayers = {0u, 0u};
    for (auto e : this->reg.view<Light, uLight>()) {
        auto [light, ulight] = this->reg.get<Light, uLight>(e);
        const Camera* cam = this->reg.try_get<Camera>(e);
        ulight = light.toStruct(
            cam == nullptr ? this->camera : *cam,
            light.type == LightType::point ? shadowMapLayers[0] : shadowMapLayers[1]);
    }
}

void App::drawSky(const Camera& cam, const Vector2f& viewport) {
    const Matrix4f view = cam.getView();
    const Matrix4f proj = cam.getProj(viewport);
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

void App::drawMeshesDepth(const Camera& cam, const Vector2f& viewport) {
    const Matrix4f view = cam.getView();
    const Matrix4f proj = cam.getProj(viewport);
    this->depthProg.Bind();
    glBindVertexArray(this->vaoMeshes); $gl_err();
    
    // Draw models in scene
    this->meshes.bind(this->depthProg, false);
    this->depthProg.SetUniformMatrix4("uTProj", proj.data());
    this->depthProg.SetUniformMatrix4("uTView", view.data());
    this->depthProg.SetUniform3("uLightPos", cam.pos.data());
    this->depthProg.SetUniform("uFarPlane", cam.far);
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

void App::drawMeshes(const Camera& cam, const Vector2f& viewport) {
    const Matrix4f view = cam.getView();
    const Matrix4f proj = cam.getProj(viewport);
    this->meshProg.Bind();
    glBindVertexArray(this->vaoMeshes); $gl_err();
    
    // Draw models in scene
    this->meshes.textures.bind(this->meshProg, "sky_cubemap", "uEnvTex");
    this->meshes.textures.bind(this->meshProg, "shadow_map", "uCubeShadowMaps");
    this->meshes.textures.bind(this->meshProg, "spot_shadow_map", "u2DShadowMaps");
    this->meshes.textures.bind(this->meshProg, "refl_maps", "uReflectionMaps");
    this->meshes.bind(this->meshProg);
    this->meshProg.SetUniformMatrix4("uTProj", proj.data());
    this->meshProg.SetUniformMatrix4("uTView", view.data());
    this->meshProg.SetUniform3("uCamPos", cam.pos.data());
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

void App::drawDebug(const Camera& cam, const Vector2f& viewport) {
    const Matrix4f view = cam.getView();
    const Matrix4f proj = cam.getProj(viewport);
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
    glBindTexture(GL_TEXTURE_2D, 0); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0); $gl_err();
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
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        transformsStorage.size() * sizeof(Matrix4f), *transformsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
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
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, this->ssboArrows); $gl_err();
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
    // Update arrow colors buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboArrowColors); $gl_err();
    auto& colorsStorage = this->reg.view<DebugColor>().storage<DebugColor>();
    if (colorsStorage.size() > 0) {
        glBufferData(GL_SHADER_STORAGE_BUFFER,
            colorsStorage.size() * sizeof(DebugColor), *colorsStorage.raw(), GL_DYNAMIC_DRAW); $gl_err();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, this->ssboArrowColors); $gl_err();
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();
}

void App::draw(float dt) {
    this->updateBuffers();

    for (const RenderPass& pass : this->renderPasses) {
        // Hide entities in mask
        for (const entt::entity& e : pass.objMask) {
            this->hidden(e, true);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, pass.fbo); $gl_err();
        // Set up render targets
        bool colorAttached = (pass.type == RenderPass::Type::final);
        bool depthAttached = (pass.type == RenderPass::Type::final);
        for (const RenderTarget& target : pass.targets) {
            switch (target.type) {
                case RenderTarget::Type::texture:
                    glFramebufferTexture2D(GL_FRAMEBUFFER, target.attachment, GL_TEXTURE_2D, target.id, 0); $gl_err();
                    break;
                case RenderTarget::Type::renderbuffer:
                    glFramebufferRenderbuffer(GL_FRAMEBUFFER, target.attachment, GL_RENDERBUFFER, target.id); $gl_err();
                    break;
            }
            if (target.type != RenderTarget::Type::renderbuffer) {
                glGenerateTextureMipmap(target.id); $gl_err();
            }
            if (target.attachment == GL_COLOR_ATTACHMENT0) {
                colorAttached = true;
            } else if (target.attachment == GL_DEPTH_ATTACHMENT) {
                depthAttached = true;
            }
        }
        if (pass.arrayTarget.attachment == GL_DEPTH_ATTACHMENT) {
            depthAttached = true;
        } else if (pass.arrayTarget.attachment == GL_COLOR_ATTACHMENT0) {
            colorAttached = true;
        }

        // Disable draw buffers if we're just doing depth
        bool depthOnly = !colorAttached;
        if (depthOnly) {
            glDrawBuffer(GL_NONE); $gl_err();
            glReadBuffer(GL_NONE); $gl_err();
        } else if (pass.type != RenderPass::Type::final) {
            glDrawBuffer(GL_COLOR_ATTACHMENT0); $gl_err();
            glReadBuffer(GL_COLOR_ATTACHMENT0); $gl_err();
        }

        // Set the viewport if it's specified, otherwise use window viewport
        Vector2f viewport = this->windowSize.cast<float>();
        if (pass.viewport) {
            viewport = *pass.viewport;
        }
        glViewport(0, 0, viewport.x(), viewport.y()); $gl_err();

        // Determine how many times the scene needs to be drawn
        size_t iters = 1u;
        switch (pass.type) {
            case RenderPass::Type::cubemap:
                iters = 6u;
                break;
            default:
                break;
        }
        for (size_t i = 0u; i < iters; i++) {
            if (!depthOnly) {
                if (pass.type == RenderPass::Type::cubemap || pass.type == RenderPass::Type::array) {
                    for (const auto e : this->reg.view<Model, ReflectionProbe>()) {
                        const auto [model, probe] = this->reg.get<Model, ReflectionProbe>(e);
                        assert(probe.layer >= 0);
                        this->hidden(e, true);
                        glFramebufferTextureLayer(
                            GL_FRAMEBUFFER, pass.arrayTarget.attachment,
                            pass.arrayTarget.id, 0, probe.layer * 6 + i); $gl_err();
                        const Camera cam = {
                            .pos = model.transformedCenter(),
                            .rot = gfx::cubeMapCameraRotations[i % 6u],
                            .fov = tau4,
                        };
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); $gl_err();
                        this->drawMeshes(cam, viewport);
                        this->hidden(e, false);
                    }
                } else {
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); $gl_err();
                    this->drawMeshes(this->camera, viewport);
                }
            } else {
                if (pass.type == RenderPass::Type::cubemap || pass.type == RenderPass::Type::array) {
                    // Draw depth faces of a cubemap, one light per layer in the array texture
                    for (const auto e : this->reg.view<Light, Camera, uLight>()) {
                        const auto [light, lightCam, ulight] = this->reg.get<Light, Camera, uLight>(e);
                        assert(ulight.shadowMapLayer >= 0);
                        GLint layer;
                        if (pass.type == RenderPass::Type::cubemap) {
                            if (light.type != LightType::point) {
                                continue;
                            }
                            layer = ulight.shadowMapLayer * 6 + i;
                        } else {
                            if (light.type != LightType::spot) {
                                continue;
                            }
                            layer = ulight.shadowMapLayer;
                        }
                        glFramebufferTextureLayer(
                            GL_FRAMEBUFFER, pass.arrayTarget.attachment,
                            pass.arrayTarget.id, 0, layer); $gl_err();
                        Camera cam(lightCam);
                        light.shadowCam(cam, i);
                        glClear(GL_DEPTH_BUFFER_BIT); $gl_err();
                        this->drawMeshesDepth(cam, viewport);
                    }
                } else {
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); $gl_err();
                    this->drawMeshes(this->camera, viewport);
                }
            }
        }

        // Generate mipmaps
        for (const RenderTarget& target : pass.targets) {
            glGenerateTextureMipmap(target.id); $gl_err();
        }
        if (pass.type == RenderPass::Type::cubemap) {
            glGenerateTextureMipmap(pass.arrayTarget.id); $gl_err();
        }


        // Unhide entities in mask
        for (const entt::entity& e : pass.objMask) {
            this->hidden(e, false);
        }
    }

    if (this->doDrawDebug) {
        this->drawDebug(this->camera, this->windowSize.cast<float>());
    }
}

void App::composeUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    ImGui::PushFont(this->ui.font);

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(500, this->windowSize.y()));
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Text("Left Click: Pan");
    ImGui::Text("Right Click: Drag Object");
    ImGui::Text("Shift + Left Click: Move light");
    ImGui::Text("Wheel: Zoom");
    ImGui::Text("1: Orbit camera");
    ImGui::Text("2: Fly camera");
    ImGui::Text("F6: Reload shaders");
    ImGui::Text(fmt::format("camera pos: {}", this->cameraControl.pos).c_str());
    ImGui::Text(fmt::format("camera rot: {}", this->cameraControl.rot).c_str());
    ImGui::Text(fmt::format("phi={} theta={}", this->cameraControl.orbitPhi(), this->cameraControl.orbitTheta()).c_str());
    Model& selectModel = this->reg.get<Model>(this->eSelectPoint);
    ImGui::Text(fmt::format("select point: {}", selectModel.pos).c_str());
    ImGui::SliderFloat("Sim Time Step", &this->simTimeStep, 0.0f, 1.0f);
    ImGui::SliderInt("Sim Iterations", &this->simTimeIters, 1, 20);

    SpringMesh& springMesh = this->reg.get<SpringMesh>(this->eSpringMesh);
    ImGui::SliderFloat("Stiffness", &springMesh.stiffness, 0.0f, 1.0f);
    ImGui::SliderFloat("Damping", &springMesh.damping, 0.0f, 0.2f);

    if (ImGui::Button("Reset Forces")) {
        springMesh.resetForces();
    }

    ImGui::Checkbox("Draw Debug", &this->doDrawDebug);

    ImGui::PopFont();
    ImGui::End();
    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();
    ImGui_ImplOpenGL3_RenderDrawData(drawData);
}

void App::hidden(entt::entity e, bool hidden) {
    auto [meshRef, objRef] = this->reg.get<MeshRef, ObjRef>(e);

    if (hidden) {
        this->vCounts.at(objRef.objID) = 0;
    } else {
        this->vCounts.at(objRef.objID) = meshRef.elemCount;
    }
}

ObjRef App::makeObj(const MeshRef& mesh) {
    ObjRef objRef;
    objRef.objID = this->vCounts.size();
    this->vCounts.push_back(mesh.elemCount);
    this->vOffsets.push_back(mesh.elemByteOffset);
    return objRef;
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

    this->reg.emplace<Model>(e, meshData);
    MeshRef& meshRef = this->reg.emplace<MeshRef>(e, meshData.ref);

    this->reg.emplace<ObjRef>(e, this->makeObj(meshRef));
    
    this->reg.emplace<ModelTransform>(e);
    return e;
}

entt::entity App::makeRigidBody(const std::string& name, const Vector3f &scale, const Vector3f& pos, const Vector3f& rot) {
    entt::entity e = this->reg.create();

    const MeshData& meshData = this->meshes.get(name);

    const Vector3f halfExtents = meshData.bbox.extents().cwiseProduct(scale);

    Model& model = this->reg.emplace<Model>(e, meshData);
    model.pos = pos;
    model.scale = scale;
    MeshRef& meshRef = this->reg.emplace<MeshRef>(e, meshData.ref);

    PhysicsBody& physicsBody = this->reg.emplace<PhysicsBody>(e);
    physicsBody.pos = pos;
    ColliderBox& collider = this->reg.emplace<ColliderBox>(e, ColliderBox{
        .halfExtents = halfExtents
    });
    this->reg.emplace<RigidBody>(e, collider);

    this->reg.emplace<ObjRef>(e, this->makeObj(meshRef));
    
    this->reg.emplace<ModelTransform>(e);

    return e;
}

entt::entity App::makeLight(const Vector3f& pos, const Vector3f& color, float intensity, float range, LightType type) {
    entt::entity e = this->makeModel(this->meshes.clone("sphere", uMaterial {
        .emissionColor = color,
        .emissionFactor = 1.0f
    }));

    Light& light = this->reg.emplace<Light>(e);
    light.pos = pos;
    light.color = color;
    light.intensity = intensity;
    light.type = type;
    light.range = range;
    light.far = 20.0f;
    light.castsShadows = true;

    Model& model = this->reg.get<Model>(e);
    model.pos = pos;
    model.scale *= 0.1f;

    this->reg.emplace<uLight>(e);

    if (light.type == LightType::point) {
        Camera& camera = this->reg.emplace<Camera>(e);
        camera.pos = pos;
        camera.fov = tau4;
        camera.near = 0.1f;
        camera.far = light.far;
        camera.projection = Camera::Projection::perspective;
    }

    return e;
}

entt::entity App::makeSpotLight(const Vector3f& pos, const Vector3f& dir, const Vector3f& color, float intensity, float spotAngle) {
    entt::entity e = this->reg.create();

    Light& light = this->reg.emplace<Light>(e);
    light.pos = pos;
    light.dir = dir;
    light.color = color;
    light.intensity = intensity;
    light.type = LightType::spot;
    light.spotAngle = spotAngle;
    light.far = 20.0f;
    light.castsShadows = true;

    this->reg.emplace<uLight>(e);

    DebugRay& debugRay = this->reg.emplace<DebugRay>(e);
    debugRay.pos = pos;
    debugRay.rot = vec3(pointSphere(dir));

    RayTransform& rayTransform = this->reg.emplace<RayTransform>(e);
    rayTransform.transform = debugRay.transform();

    DebugColor& debugColor = this->reg.emplace<DebugColor>(e);
    debugColor.color = {1.0f, 0.7f, 0.2f, 1.0f};

    Camera& camera = this->reg.emplace<Camera>(e);
    camera.pos = pos;
    camera.rot = dirToRot(dir);
    camera.fov = light.spotAngle;
    camera.near = 0.1f;
    camera.far = light.far;
    camera.projection = Camera::Projection::perspective;

    return e;
}