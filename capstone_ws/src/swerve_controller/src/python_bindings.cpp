// Python 바인딩: import swerve_kinematics — C++ 노드와 같은 계산을 Python(시뮬·시험·시각화)에서 그대로 씀.
//
//   import swerve_kinematics as sk
//   cfg = sk.ControllerConfig()
//   cfg.modules = [(0.1048, -0.1556), (-0.1048, -0.1556), (-0.1048, 0.1556), (0.1048, 0.1556)]  # FR RR RL FL
//   cfg.lo, cfg.hi = [-2.967] * 4, [2.967] * 4
//   ctl = sk.Controller(cfg)
//   r = ctl.step(0.2, 0.0, 0.0, actual_angle=[0.0] * 4, actual_ok=True)
//   r.angle, r.speed, r.wheel_speed, r.gated
//   sk.fk(cfg.modules, angle, speed)  -> (vx, vy, omega)
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <utility>
#include <vector>

#include "swerve_controller/kinematics.hpp"

namespace py = pybind11;
using namespace swerve;

namespace {
using XY = std::pair<double, double>;

std::array<XY, N> modules_to_py(const Modules& m) {
  std::array<XY, N> out;
  for (std::size_t i = 0; i < N; ++i) out[i] = {m[i].x, m[i].y};
  return out;
}
Modules modules_from_py(const std::array<XY, N>& in) {
  Modules m;
  for (std::size_t i = 0; i < N; ++i) m[i] = {in[i].first, in[i].second};
  return m;
}

struct PyStep {
  Arr angle{}, speed{}, wheel_speed{};
  bool gated = false;
  bool stopping = false;
};
}  // namespace

PYBIND11_MODULE(swerve_kinematics, m) {
  m.doc() = "4모듈 스워브 기구학 (swerve_controller C++ 코어). 순서 [FR, RR, RL, FL], 각 [rad], 속도 [m/s]";
  m.attr("N") = N;

  py::class_<ControllerConfig>(m, "ControllerConfig")
      .def(py::init<>())
      .def_property(
          "modules", [](const ControllerConfig& c) { return modules_to_py(c.modules); },
          [](ControllerConfig& c, const std::array<XY, N>& v) { c.modules = modules_from_py(v); },
          "조향축 위치 [(x, y)] ×4 [m], 회전중심 기준 x 전진 / y 좌측")
      .def_readwrite("lo", &ControllerConfig::lo, "조향 하한 ×4 [rad]")
      .def_readwrite("hi", &ControllerConfig::hi, "조향 상한 ×4 [rad]")
      .def_readwrite("max_wheel_mps", &ControllerConfig::max_wheel_mps)
      .def_readwrite("min_steer_mps", &ControllerConfig::min_steer_mps, "이보다 느린 모듈은 조향 유지, <= 0 이면 끔")
      .def_readwrite("unwind_rad", &ControllerConfig::unwind_rad)
      .def_readwrite("slow_mps", &ControllerConfig::slow_mps)
      .def_readwrite("gate_trigger_rad", &ControllerConfig::gate_trigger_rad, "<= 0 이면 게이트 끔")
      .def_readwrite("gate_release_rad", &ControllerConfig::gate_release_rad)
      .def_readwrite("gate_max_hold_ticks", &ControllerConfig::gate_max_hold_ticks)
      .def_readwrite("flip_stop_rad", &ControllerConfig::flip_stop_rad, "<= 0 이면 정지 후 조향 끔")
      .def_readwrite("stop_speed_mps", &ControllerConfig::stop_speed_mps)
      .def_readwrite("est_decel_mps2", &ControllerConfig::est_decel_mps2)
      .def_readwrite("dt", &ControllerConfig::dt)
      .def_readwrite("max_steer_rate", &ControllerConfig::max_steer_rate, "조향 명령 회전 속도 제한 [rad/s], <= 0 이면 끔");

  py::class_<PyStep>(m, "StepResult")
      .def_readonly("angle", &PyStep::angle, "조향 명령 [rad] (±한계로 접은 뒤)")
      .def_readonly("speed", &PyStep::speed, "바퀴 속도 [m/s] (게이트 적용 전)")
      .def_readonly("wheel_speed", &PyStep::wheel_speed, "바퀴 속도 [m/s] (게이트 적용 후 — 실제로 보낼 값)")
      .def_readonly("gated", &PyStep::gated)
      .def_readonly("stopping", &PyStep::stopping, "큰 조향 전환 전에 멈추는 중")
      .def("__repr__", [](const PyStep& r) {
        return "StepResult(angle=" + py::repr(py::cast(r.angle)).cast<std::string>() +
               ", wheel_speed=" + py::repr(py::cast(r.wheel_speed)).cast<std::string>() +
               ", gated=" + (r.gated ? "True" : "False") + ")";
      });

  py::class_<Controller>(m, "Controller")
      .def(py::init<ControllerConfig>(), py::arg("cfg"))
      .def("seed", &Controller::seed, py::arg("actual_angle"), "이력을 실제 조향각으로 초기화 (FREE → 활성 때)")
      .def("reset", &Controller::reset)
      .def(
          "step",
          [](Controller& c, double vx, double vy, double omega, const Arr& actual_angle, bool actual_ok,
             const Arr& wheel_speed, bool wheel_ok) {
            const auto r = c.step({vx, vy, omega}, actual_angle, actual_ok, wheel_speed, wheel_ok);
            PyStep p;
            for (std::size_t i = 0; i < N; ++i) {
              p.angle[i] = r.cmd[i].angle;
              p.speed[i] = r.cmd[i].speed;
            }
            p.wheel_speed = r.wheel_speed;
            p.gated = r.gated;
            p.stopping = r.stopping;
            return p;
          },
          py::arg("vx"), py::arg("vy"), py::arg("omega"), py::arg("actual_angle") = Arr{},
          py::arg("actual_ok") = false, py::arg("wheel_speed") = Arr{}, py::arg("wheel_ok") = false,
          "한 제어주기: IK → ±한계 접기 → (큰 조향 전환이면 먼저 정지) → 정렬 게이트")
      .def_property_readonly("gate_pending", [](const Controller& c) { return c.gate().pending(); });

  m.def(
      "fk",
      [](const std::array<XY, N>& modules, const Arr& angle, const Arr& speed) {
        const auto t = fk_compute(modules_from_py(modules), angle, speed);
        return py::make_tuple(t.vx, t.vy, t.omega);
      },
      py::arg("modules"), py::arg("angle"), py::arg("speed"),
      "정기구학 (최소제곱): 조향각 [rad] + 바퀴 속도 [m/s] → (vx, vy, omega)");
}
