#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>

/**
 * Board-agnostic RGB status LED (single WS2812B / NeoPixel).
 *
 * Instead of showing a single highest-priority state, the LED shows the SET of
 * currently-active conditions: each active condition contributes one colour, and
 * the LED shows them one at a time in a loop (green -> blue -> yellow -> ... ->
 * repeat), each colour breathing from near-off to full and back (symmetric fade
 * up/down). Colours are shown discretely, not blended, so there are no ambiguous
 * mid-hues. A colour is held for its whole breath; a status change that alters
 * the set is only picked up at the breath's low point, so colours never snap at
 * high brightness. So several concerns are visible at a glance over one cycle.
 *
 *   GREEN  - network connected (WiFi or cellular data up)
 *   BLUE   - car link (BLE/CAN) connecting / retrying / not connected
 *   RED    - error: no network at all, or an MQTT/ABRP/API upload error
 *            (auth rejected, bad token, server/transport failure)
 *   YELLOW - no GPS fix
 * If no condition is active the LED is off.
 *
 * Rendering runs in a dedicated low-priority FreeRTOS task. The task also creates
 * and owns the NeoPixel object: the Adafruit ESP32 backend binds the RMT
 * peripheral to the task that drives it, so creating it on the main loop and
 * calling show() from the task raced the RMT hardware and caused flicker. The
 * task yields with vTaskDelay between frames, so it adds no measurable load and
 * never slows modem/CAN/GPS I/O. update() only caches the latest flags under a
 * short critical section; a status update seen a second late is harmless.
 *
 * NOTE: this board's onboard LED is RGB byte order, not GRB.
 */
class StatusLed
{
public:
  struct DeviceStatus
  {
    bool netConnected = false;  // WiFi or cellular data up          -> green
    bool commConnecting = false; // car BLE/CAN not up / retrying     -> blue
    bool error = false;         // no net, or upload/auth error      -> red
    bool noGpsFix = false;      // no usable GPS fix                 -> yellow
  };

  // pin: GPIO for the WS2812B. brightness: 0..255 (0 disables output).
  void begin(uint8_t pin, uint8_t brightness = 40);
  void setBrightness(uint8_t brightness);
  void setEnabled(bool on);
  // Cache the latest condition flags. Cheap and non-blocking: the LED is
  // rendered by the internal task, so this can be called at the (slow) main loop
  // rate. A late update (a second or two) is harmless.
  void update(const DeviceStatus &s);

private:
  struct Rgb
  {
    uint8_t r, g, b;
  };

  static constexpr uint8_t kMaxColors = 4;

  static void renderTask(void *arg); // FreeRTOS task entry
  void renderOnce();
  void show(const Rgb &c, uint8_t scale);
  // Build the active-colour list from the cached flags. Returns the count.
  uint8_t buildColors(const DeviceStatus &st, Rgb out[kMaxColors]) const;

  void *strip = nullptr;         // Adafruit_NeoPixel* (opaque to keep this header light)
  TaskHandle_t taskHandle = nullptr;
  uint8_t ledPin = 0;

  // Shared state written by update()/setters (caller task) and read by the
  // render task. Guarded by a short critical section.
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  DeviceStatus status;
  uint8_t bright = 40;
  bool enabled = true;
  volatile bool started = false;

  // Render-task-only state (not shared, no lock). The current breath holds one
  // colour for its whole period and only re-samples the active set at the low
  // point of the breath, so a status change never snaps the colour at high
  // brightness — it is picked up on the next breath instead.
  Rgb breathColor{0, 0, 0};   // colour currently being breathed
  bool breathActive = false;  // whether breathColor is valid / a breath is in progress
  uint32_t breathStartMs = 0; // millis() when the current breath began
  uint8_t breathIndex = 0;    // position within the active set, to advance each breath
};
