#include <cstdio>
#include <cstring>
#include <string>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  std::string model_path = "models/mjcf/world.xml";
  if (argc > 1) {
    model_path = argv[1];
  }

  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to initialize GLFW\n");
    return 1;
  }

  GLFWwindow* win = glfwCreateWindow(1280, 720, "ScratchArm 6-DoF Sim", nullptr, nullptr);
  if (!win) {
    std::fprintf(stderr, "Failed to create window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  char error[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "Failed to load model: %s\nError: %s\n", model_path.c_str(), error);
    glfwTerminate();
    return 1;
  }
  mjData* d = mj_makeData(m);

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

  if (m->ncam > 0) {
    mjv_defaultFreeCamera(m, &cam);
  }

  while (!glfwWindowShouldClose(win)) {
    mj_step(m, d);

    glfwPollEvents();
    int fb_w, fb_h;
    glfwGetFramebufferSize(win, &fb_w, &fb_h);
    mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
    mjrRect viewport = {0, 0, fb_w, fb_h};
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(win);
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
  return 0;
}
