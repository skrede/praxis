#include "praxis/scene/visualizer.h"

#include "windows/log_window.h"
#include "windows/preset_window.h"
#include "windows/view_gizmo_window.h"
#include "windows/stepped_work_window.h"

#include <memory>
#include <utility>

namespace praxis::scene {

namespace {

constexpr float frustum_size       = 5.f;
constexpr unsigned int grid_extent = 10u;

}

void visualizer::setup_scene()
{
    add_scene_grid();
    add_camera();
    add_scene_lights();
    add_imgui_windows();
}

void visualizer::add_scene_grid()
{
    const auto extent    = static_cast<float>(grid_extent);
    auto plane           = threepp::Mesh::create(threepp::PlaneGeometry::create(extent, extent), threepp::ShadowMaterial::create());
    plane->rotation.y    = -threepp::math::PI / 2.f;
    plane->receiveShadow = true;

    m_grid_helper             = threepp::GridHelper::create(grid_extent, grid_extent, threepp::Color::yellowgreen);
    m_grid_helper->rotation.y = threepp::math::PI / 2.f;
    plane->add(m_grid_helper);
    m_scene->add(m_grid_helper);
}

void visualizer::add_camera()
{
    if(m_projection == projection::perspective)
        m_camera = threepp::PerspectiveCamera::create(75.f, m_canvas->aspect(), 0.1f, 100.f);
    else
        m_camera = threepp::OrthographicCamera::create(-(frustum_size * m_canvas->aspect()) / 2.f, (frustum_size * m_canvas->aspect()) / 2.f, frustum_size / 2.f, -frustum_size / 2.f,
                                                       0.001f, 100.f);
    m_camera->position.set(0.f, 1.25f, 1.f);
    m_controls = std::make_unique<threepp::OrbitControls>(*m_camera, *m_canvas);
}

void visualizer::add_scene_lights()
{
    m_scene->add(threepp::HemisphereLight::create(threepp::Color::aliceblue, threepp::Color::grey));
}

void visualizer::add_imgui_windows()
{
    if(m_messages == nullptr)
    {
        m_messages = std::make_shared<log_buffer>(default_log_capacity);
        install_log_sink(m_messages);
    }

    m_imgui->add_window(std::make_shared<log_window>("Messages", m_messages));
    m_imgui->add_window(std::make_shared<preset_window>("Presets", *this));
    m_imgui->add_window(std::make_shared<view_gizmo_window>("ViewGizmo", *m_camera, *m_controls));
    m_imgui->add_window(std::make_shared<stepped_work_window>("Stepped work", *this));
}

void visualizer::capture_input()
{
    m_capture                     = std::make_unique<threepp::IOCapture>();
    m_capture->preventMouseEvent  = [] { return ImGui::GetIO().WantCaptureMouse; };
    m_capture->preventScrollEvent = [] { return ImGui::GetIO().WantCaptureMouse; };
    m_canvas->setIOCapture(m_capture.get());
    m_canvas->onWindowResize([this](threepp::WindowSize size) { resize(size); });
}

void visualizer::resize(threepp::WindowSize size)
{
    if(m_projection == projection::perspective)
        static_cast<threepp::PerspectiveCamera &>(*m_camera).aspect = size.aspect();
    else
    {
        auto &camera  = static_cast<threepp::OrthographicCamera &>(*m_camera);
        camera.left   = -frustum_size * size.aspect() / 2.f;
        camera.right  = frustum_size * size.aspect() / 2.f;
        camera.top    = frustum_size / 2.f;
        camera.bottom = -frustum_size / 2.f;
    }
    m_camera->updateProjectionMatrix();
    m_renderer->setSize(size);
}

}
