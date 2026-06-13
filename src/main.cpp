#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>

#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"

#include "Algorithm/Kinematics.h"

#ifndef GL_READ_FRAMEBUFFER
#define GL_READ_FRAMEBUFFER 0x8CA8
#endif
#ifndef GL_DRAW_FRAMEBUFFER
#define GL_DRAW_FRAMEBUFFER 0x8CA9
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_RGBA8
#define GL_RGBA8 0x8058
#endif

using PFN_glGenFramebuffers = void (*)(GLsizei, GLuint *);
using PFN_glDeleteFramebuffers = void (*)(GLsizei, const GLuint *);
using PFN_glBindFramebuffer = void (*)(GLenum, GLuint);
using PFN_glFramebufferTexture2D = void (*)(GLenum, GLenum, GLenum, GLuint, GLint);
using PFN_glCheckFramebufferStatus = GLenum (*)(GLenum);
using PFN_glBlitFramebuffer = void (*)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);

struct GLFbo
{
    PFN_glGenFramebuffers glGenFramebuffers = nullptr;
    PFN_glDeleteFramebuffers glDeleteFramebuffers = nullptr;
    PFN_glBindFramebuffer glBindFramebuffer = nullptr;
    PFN_glFramebufferTexture2D glFramebufferTexture2D = nullptr;
    PFN_glCheckFramebufferStatus glCheckFramebufferStatus = nullptr;
    PFN_glBlitFramebuffer glBlitFramebuffer = nullptr;

    bool load()
    {
        glGenFramebuffers = reinterpret_cast<PFN_glGenFramebuffers>(glfwGetProcAddress("glGenFramebuffers"));
        glDeleteFramebuffers = reinterpret_cast<PFN_glDeleteFramebuffers>(glfwGetProcAddress("glDeleteFramebuffers"));
        glBindFramebuffer = reinterpret_cast<PFN_glBindFramebuffer>(glfwGetProcAddress("glBindFramebuffer"));
        glFramebufferTexture2D =
            reinterpret_cast<PFN_glFramebufferTexture2D>(glfwGetProcAddress("glFramebufferTexture2D"));
        glCheckFramebufferStatus =
            reinterpret_cast<PFN_glCheckFramebufferStatus>(glfwGetProcAddress("glCheckFramebufferStatus"));
        glBlitFramebuffer = reinterpret_cast<PFN_glBlitFramebuffer>(glfwGetProcAddress("glBlitFramebuffer"));
        return glGenFramebuffers && glBindFramebuffer && glFramebufferTexture2D && glBlitFramebuffer;
    }
};

struct ViewportTexture
{
    GLuint fbo = 0;
    GLuint tex = 0;
    int w = 0;
    int h = 0;

    void ensure(const GLFbo &gl, int width, int height)
    {
        if (width <= 0 || height <= 0)
            return;
        if (width == w && height == h && tex)
            return;

        destroy(gl);
        w = width;
        h = height;

        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);

        gl.glGenFramebuffers(1, &fbo);
        gl.glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        gl.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
        if (gl.glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::fprintf(stderr, "Viewport FBO is incomplete\n");
        gl.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void destroy(const GLFbo &gl)
    {
        if (fbo)
            gl.glDeleteFramebuffers(1, &fbo);
        if (tex)
            glDeleteTextures(1, &tex);
        fbo = tex = 0;
        w = h = 0;
    }
};

struct CameraController
{
    float viewport_h = 1.0f;
    bool hovered = false;

    void apply(const mjModel *m, const mjvScene *scn, mjvCamera *cam, ImGuiIO &io)
    {
        if (!hovered || viewport_h <= 0.0f)
            return;

        if (io.MouseWheel != 0.0f)
            mjv_moveCamera(m, mjMOUSE_ZOOM, 0.0, -static_cast<double>(io.MouseWheel) * 0.1, scn, cam);

        const float dx = io.MouseDelta.x / viewport_h;
        const float dy = io.MouseDelta.y / viewport_h;

        int action = mjMOUSE_NONE;
        if (io.MouseDown[ImGuiMouseButton_Left])
            action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else if (io.MouseDown[ImGuiMouseButton_Right])
            action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (io.MouseDown[ImGuiMouseButton_Middle])
            action = mjMOUSE_ZOOM;

        if (action != mjMOUSE_NONE && (dx != 0.0f || dy != 0.0f))
            mjv_moveCamera(m, action, dx, dy, scn, cam);
    }
};

int main(int argc, char **argv)
{
    std::string model_path = "models/mjcf/world.xml";
    if (argc > 1)
        model_path = argv[1];

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    GLFWwindow *win = glfwCreateWindow(1280, 720, "ScratchArm 6-DoF Sim", nullptr, nullptr);
    if (!win) {
        std::fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    char error[1024] = "";
    mjModel *m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::fprintf(stderr, "Failed to load model: %s\nError: %s\n", model_path.c_str(), error);
        glfwTerminate();
        return 1;
    }
    mjData *d = mj_makeData(m);
    mj_forward(m, d);

    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    mjvScene scn;
    mjrContext con;
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 100000);
    if (m->ncam > 0)
        mjv_defaultFreeCamera(m, &cam);

    arm_kin::ArmKinematics kin(m);
    if (!kin.valid())
        std::fprintf(stderr, "Warning: kinematics model incomplete (expected 6 named joints + End_Link)\n");

    GLFbo glfbo;
    if (!glfbo.load()) {
        std::fprintf(stderr, "Failed to load GL framebuffer functions\n");
        return 1;
    }
    ViewportTexture vp;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    constexpr float kFontSize = 20.0f;
    const char *kFontCJK = "/usr/share/fonts/windows/msyh.ttc"; // 微软雅黑
    const char *kFontLatin = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
    const auto file_exists = [](const char *p) {
        FILE *f = std::fopen(p, "r");
        if (!f)
            return false;
        std::fclose(f);
        return true;
    };
    ImFontConfig font_cfg;
    if (file_exists(kFontCJK))
        io.Fonts->AddFontFromFileTTF(kFontCJK, kFontSize, &font_cfg, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
    else if (file_exists(kFontLatin))
        io.Fonts->AddFontFromFileTTF(kFontLatin, kFontSize, &font_cfg);

    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    std::array<float, arm_kin::ArmKinematics::kNumJoints> q_target{};
    float ik_pos[3] = {0.0f, 0.0f, 0.0f};
    float ik_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // identity rotation (w first)
    arm_kin::IKResult last_ik{};
    CameraController camera;
    bool dock_layout_built = false;

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (kin.valid()) {
            for (int i = 0; i < arm_kin::ArmKinematics::kNumJoints; ++i) {
                const double lo = kin.jointLow(i), hi = kin.jointHigh(i);
                double qi = static_cast<double>(q_target[i]);
                qi = std::clamp(qi, lo, hi);
                q_target[i] = static_cast<float>(qi);
                d->qpos[kin.qposAdr(i)] = qi;
            }
        }
        mj_forward(m, d);

        camera.apply(m, &scn, &cam, io);
        mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);

        const ImGuiViewport *main_vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(main_vp->WorkPos);
        ImGui::SetNextWindowSize(main_vp->WorkSize);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##DockSpace",
                     nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                         ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoDocking);
        ImGui::PopStyleVar(2);

        const ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

        if (!dock_layout_built && ImGui::DockBuilderGetNode(dockspace_id) == nullptr) {
            dock_layout_built = true;
            ImGui::DockBuilderRemoveNode(dockspace_id);
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace_id, main_vp->WorkSize);
            ImGuiID dock_left = 0, dock_right = 0;
            ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.32f, &dock_left, &dock_right);
            ImGui::DockBuilderDockWindow("Control", dock_left);
            ImGui::DockBuilderDockWindow("Viewport", dock_right);
            ImGui::DockBuilderFinish(dockspace_id);
        }
        ImGui::End();

        ImGui::Begin("Control");

        ImGui::TextUnformatted("Joint Angles (rad)");
        ImGui::Spacing();
        for (int i = 0; i < arm_kin::ArmKinematics::kNumJoints; ++i) {
            ImGui::PushID(i);
            const double lo = kin.valid() ? kin.jointLow(i) : -6.28;
            const double hi = kin.valid() ? kin.jointHigh(i) : 6.28;

            ImGui::AlignTextToFramePadding();
            ImGui::Text("%-14s", arm_kin::ArmKinematics::kJointNames[i]);
            ImGui::SameLine(150);

            // Slider and input box are both bound to q_target[i] → they stay in sync.
            ImGui::SetNextItemWidth(200);
            ImGui::SliderFloat("##slider", &q_target[i], static_cast<float>(lo), static_cast<float>(hi), "%.3f");
            ImGui::SameLine(0, 8);
            ImGui::SetNextItemWidth(90);
            ImGui::InputFloat("##input", &q_target[i], 0.0f, 0.0f, "%.3f");
            ImGui::PopID();
        }

        ImGui::Spacing();
        ImGui::SeparatorText("Forward Kinematics (live)");
        arm_kin::Pose ee{};
        if (kin.valid()) {
            std::array<double, arm_kin::ArmKinematics::kNumJoints> qd{};
            for (int i = 0; i < arm_kin::ArmKinematics::kNumJoints; ++i)
                qd[i] = q_target[i];
            kin.forward(qd.data(), ee);
        }
        ImGui::Text("end-effector pos : (%.3f, %.3f, %.3f)", ee.pos[0], ee.pos[1], ee.pos[2]);
        ImGui::Text(
            "end-effector quat: (%.3f, %.3f, %.3f, %.3f)  [w x y z]", ee.quat[0], ee.quat[1], ee.quat[2], ee.quat[3]);

        ImGui::SeparatorText("Inverse Kinematics (6-DoF pose)");
        if (kin.valid()) {
            ImGui::InputFloat3("target pos (xyz)", ik_pos, "%.4f");
            ImGui::InputFloat4("target quat (wxyz)", ik_quat, "%.4f");

            if (ImGui::Button("Solve IK")) {
                std::array<double, arm_kin::ArmKinematics::kNumJoints> qseed{};
                for (int i = 0; i < arm_kin::ArmKinematics::kNumJoints; ++i)
                    qseed[i] = q_target[i];
                arm_kin::Pose tgt;
                for (int i = 0; i < 3; ++i)
                    tgt.pos[i] = ik_pos[i];
                for (int i = 0; i < 4; ++i)
                    tgt.quat[i] = ik_quat[i];

                last_ik = kin.inverse(tgt, qseed.data(), 200, 1e-4, 1e-3, 0.05);
                if (last_ik.converged)
                    for (int i = 0; i < arm_kin::ArmKinematics::kNumJoints; ++i)
                        q_target[i] = static_cast<float>(qseed[i]);
            }
            ImGui::SameLine();
            if (ImGui::Button("Grab current pose")) {
                for (int i = 0; i < 3; ++i)
                    ik_pos[i] = static_cast<float>(ee.pos[i]);
                for (int i = 0; i < 4; ++i)
                    ik_quat[i] = static_cast<float>(ee.quat[i]);
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset pose"))
                q_target.fill(0.0f);

            const char *status = last_ik.converged ? "converged" : (last_ik.iters > 0 ? "no convergence" : "—");
            ImGui::TextDisabled("status: %s | iters %d | pos_err %.4f m | rot_err %.4f rad",
                                status,
                                last_ik.iters,
                                last_ik.pos_error,
                                last_ik.rot_error);
        } else {
            ImGui::TextDisabled("Kinematics unavailable for this model.");
        }

        ImGui::SeparatorText("Viewport Controls");
        ImGui::TextDisabled("L-drag rotate | R-drag pan | wheel/middle zoom | Shift = horizontal");

        ImGui::End();

        ImGui::Begin("Viewport");
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const int vw = std::max(1, static_cast<int>(avail.x));
        const int vh = std::max(1, static_cast<int>(avail.y));

        vp.ensure(glfbo, vw, vh);

        // Render MuJoCo into its offscreen buffer, then blit into our texture.
        mjr_resizeOffscreen(vw, vh, &con);
        mjr_setBuffer(mjFB_OFFSCREEN, &con);
        const mjrRect rect = {0, 0, vw, vh};
        mjr_render(rect, &scn, &con);

        glfbo.glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
        glfbo.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, vp.fbo);
        glfbo.glBlitFramebuffer(0, 0, vw, vh, 0, 0, vw, vh, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glfbo.glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (vp.tex)
            ImGui::Image(static_cast<ImTextureID>(vp.tex), avail, ImVec2(0, 1), ImVec2(1, 0));

        camera.hovered = ImGui::IsItemHovered();
        camera.viewport_h = static_cast<float>(vh);

        ImGui::TextDisabled("%dx%d", vw, vh);
        ImGui::End();

        ImGui::Render();
        int fb_w, fb_h;
        glfwGetFramebufferSize(win, &fb_w, &fb_h);
        glViewport(0, 0, fb_w, fb_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    vp.destroy(glfbo);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
