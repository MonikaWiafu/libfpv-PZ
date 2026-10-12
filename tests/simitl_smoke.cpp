#include "simitl.h"
#include "bf_bridge.h"
#include <chrono>
#include <cstdio>
#include <cstring>
static void feed(StateInput& in, const StateOutput& o) {
  float x=o.orientation.x,y=o.orientation.y,z=o.orientation.z,w=o.orientation.w;
  in.rotation[0] = {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)}; in.rotation[1] = {2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)}; in.rotation[2] = {2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
  in.linearVelocity = o.linearVelocity; in.angularVelocity = o.angularVelocity;
}
int main() {
  StateInit init{};
  init.quadMass = 0.11f+4*0.038f+4*0.004f+0.211f; init.propMaxRpm = 33000; init.propInertia = 8.1e-6f; init.propTorqueFactor = 0.0182f;
  init.propAFactor = 1.331e-8f; init.propThrustFactor = {-3.62e-5f, -0.1275f, 18.8f}; init.propBladeCount = 3;
  init.quadInvInertia = {550, -310, 550}; init.quadBatCellCount = 6; init.quadBatCapacity = 1100; init.quadBatCapacityCharged = 1130; init.maxVoltageSag = 1.05f;
  init.motorRth = 7; init.motorCth = 19; init.motorMaxT = 128; init.ambientTemp = 25; init.frameDragConstant = 1.2f; init.frameDragArea = {0.0097f, 0.0081f, 0.0098f};
  float d = 0.08f; Vec3F pos[4] = {{d,0.005f,-d},{d,0.005f,d},{-d,0.005f,-d},{-d,0.005f,d}}; float dir[4] = {-1,1,1,-1};
  for (int i = 0; i < 4; i++) { init.motorKV[i] = 1900; init.motorR[i] = 0.105f; init.motorI0[i] = 1.2f; init.quadMotorPos[i] = pos[i]; init.quadMotorDir[i] = dir[i]; }
  std::strcpy((char*)init.eepromName, "smoke_eeprom.bin");
  simitl_init(init);
  char* out = nullptr;
  int r = bfb_cli_run("set motor_pwm_protocol = PWM\naux 0 0 0 1700 2100 0 0\nsave noreboot\nget motor_pwm_protocol\naux", &out);
  std::fprintf(stderr, "cli r=%d\n---\n%s\n---\n", r, out); bfb_free(out);
  StateInput in{}; in.rotation[0] = {1,0,0}; in.rotation[1] = {0,1,0}; in.rotation[2] = {0,0,1}; in.vbat = 25.2f;
  for (int i = 0; i < 8; i++) in.rcData[i] = -1.f; in.rcData[0] = in.rcData[1] = in.rcData[3] = 0.f;
  float py = 0; auto t0 = std::chrono::steady_clock::now(); int frames = 10000; const float dt = 0.001f;
  for (int f = 0; f < frames; f++) {
    in.delta = dt; in.rcData[4] = f > 6000 ? 1.f : -1.f; in.rcData[2] = f > 6500 ? -0.30f : -1.f;
    in.contact = py <= 0.f; 
    simitl_update(in); StateOutput o = simitl_get_state(); feed(in, o);
    py += o.linearVelocity.y * dt; if (py < 0) { py = 0; if (in.linearVelocity.y < 0) in.linearVelocity.y = 0; }
    in.position.y = py;
    if (f % 500 == 0) std::fprintf(stderr, "t=%.1f armed=%d flags=0x%x rpm=%.0f vy=%.2f y=%.2f vbat=%.2f\n", f*dt, bfb_is_armed(), bfb_arming_disable_flags(), o.motorRpm[0], o.linearVelocity.y, py, o.batteryVoltage);
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::fprintf(stderr, "wall %.1f ms for %.1f s sim -> %.1f us per 1ms step\n", ms, frames*dt, ms*1000/frames);
  simitl_stop();
  return 0;
}
