#include <csignal>
#include <filesystem>
#include <iostream>
#include <app.hpp>
#include <gleq.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/formatter.h>
#include <spdlog/fmt/fmt.h>
#include <cxxopts.hpp>

namespace spdlog::sinks {
    template<typename Mutex>
    class error_proxy_sink : public base_sink<Mutex> {
    /* Thanks to @tt4g for this class (https://github.com/gabime/spdlog/issues/1363#issuecomment-567068416) */
    private:
        using BaseSink = base_sink<Mutex>;

        std::shared_ptr<sink> sink_;

    public:
        explicit error_proxy_sink(std::shared_ptr<sink> sink) : sink_(sink){}

        error_proxy_sink(const error_proxy_sink&) = delete;
        error_proxy_sink& operator=(const error_proxy_sink&) = delete;
    protected:
        void sink_it_(const spdlog::details::log_msg &msg) override {
            if (sink_->should_log(msg.level)) {
                sink_->log(msg);
            }
            if (spdlog::level::err == msg.level) {
                std::raise(SIGINT);
            }
        }

        void flush_() override {
            sink_->flush();
        }

        void set_pattern_(const std::string &pattern) override {
            set_formatter_(spdlog::details::make_unique<spdlog::pattern_formatter>(pattern));
        }

        void set_formatter_(std::unique_ptr<spdlog::formatter> sink_formatter) override {
            BaseSink::formatter_ = std::move(sink_formatter);

            sink_->set_formatter(BaseSink::formatter_->clone());
        }
    };

    using error_proxy_sink_mt = error_proxy_sink<std::mutex>;
    using error_proxy_sink_st = error_proxy_sink<spdlog::details::null_mutex>;
}

int main(int argc, char const *argv[])
{
    assert(fs::exists("resources"));

    // Parse arguments
    cxxopts::Options cliOptions("rendertool", "A tool for rendering 3D scenes");
    cliOptions.add_options()
        ("m,model", "Path to .obj file to render into the scene",
            cxxopts::value<std::string>()->default_value("resources/models/yoda/yoda.obj"))
        ("h,help", "Prints help");
    cxxopts::ParseResult opts = cliOptions.parse(argc, argv);
    if (opts.count("help")) {
        std::cout << cliOptions.help() << std::endl;
        return 0;
    }

    // Setup logging
    auto stderrSink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    auto errSink = std::make_shared<spdlog::sinks::error_proxy_sink_mt>(stderrSink);
    auto formatter = std::make_unique<spdlog::pattern_formatter>("%^%L%$> %v");
    auto logger = std::make_shared<spdlog::logger>("logger", errSink);
    logger->set_formatter(std::move(formatter));
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::trace);
    spdlog::flush_on(spdlog::level::trace);

    // Init GLFW
    if (!glfwInit()) {
        spdlog::error("GLFW3 failed to initialize!");
        return -1;
    }
    glfwSetErrorCallback([](int error, const char* description) {
        spdlog::error("GLFW error [{}] {}", error, description);
    });

    gleqInit();

    App app(opts);

    // Get and display version information
	std::string renderer((char*)glGetString(GL_RENDERER));
	std::string version((char*)glGetString(GL_VERSION));
    spdlog::info("OpenGL {}, {}", version, renderer);
    
    // Render loop
	app.run();
    return 0;
}
