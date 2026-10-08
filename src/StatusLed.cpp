#include "StatusLed.h"
#include <Adafruit_NeoPixel.h>

// Render cadence. 33 Hz (30 ms) is smooth to the eye. The task sleeps
// (vTaskDelay) between frames, so it costs almost nothing; each frame is one
// single-pixel NeoPixel show() (~30 us of disabled interrupts), far too short
// to disturb modem/CAN/GPS I/O.
static constexpr TickType_t kRenderDelayTicks = pdMS_TO_TICKS(30);

// Duration each active colour is shown before advancing to the next in the set,
// and the period of its breathe (near-off -> full -> near-off, symmetric). A
// set of N colours cycles in N * this. 3200 ms is a slow, calm breathe.
static constexpr uint32_t kBreathPeriodMs = 3200;

void StatusLed::begin(uint8_t pin, uint8_t brightness)
{
  ledPin = pin;
  bright = brightness;

  // The NeoPixel object is created and driven ENTIRELY inside the render task:
  // the Adafruit ESP32 backend binds the RMT peripheral to the task that drives
  // it, so creating it here and calling show() from the task raced the RMT
  // hardware and produced flicker/stuck colours. We only stash config here.
  if (taskHandle == nullptr)
  {
    xTaskCreate(&StatusLed::renderTask, "statusled", 2560, this, 1, &taskHandle);
  }
}

void StatusLed::setBrightness(uint8_t brightness)
{
  portENTER_CRITICAL(&mux);
  bright = brightness;
  portEXIT_CRITICAL(&mux);
}

void StatusLed::setEnabled(bool on)
{
  portENTER_CRITICAL(&mux);
  enabled = on;
  portEXIT_CRITICAL(&mux);
}

void StatusLed::update(const DeviceStatus &s)
{
  portENTER_CRITICAL(&mux);
  status = s;
  portEXIT_CRITICAL(&mux);
}

void StatusLed::renderTask(void *arg)
{
  StatusLed *self = static_cast<StatusLed *>(arg);

  // RGB order (not GRB) for this board's onboard LED.
  Adafruit_NeoPixel *s = new Adafruit_NeoPixel(1, self->ledPin, NEO_RGB + NEO_KHZ800);
  s->begin();
  s->clear();
  s->show();
  self->strip = s;
  self->started = true;

  for (;;)
  {
    self->renderOnce();
    vTaskDelay(kRenderDelayTicks);
  }
}

void StatusLed::show(const Rgb &c, uint8_t scale)
{
  if (!started || strip == nullptr)
  {
    return;
  }
  Adafruit_NeoPixel *s = static_cast<Adafruit_NeoPixel *>(strip);

  // Apply the global brightness and the per-frame pulse scale together, in one
  // higher-precision multiply per channel. We deliberately do NOT use NeoPixel's
  // setBrightness(): it rescales with a lossy (v*(b+1))>>8 step, and chaining
  // that after our own pulse scale compounded the rounding so hard that at low
  // global brightness the breath collapsed to a few visible levels (looked like
  // a 3-4 Hz step rather than a smooth fade). Folding both factors into one
  // 0..65025 divisor keeps the full range of levels at any brightness.
  uint8_t gb;
  portENTER_CRITICAL(&mux);
  gb = bright;
  portEXIT_CRITICAL(&mux);

  const uint32_t combined = (uint32_t)scale * gb; // 0..65025
  const uint8_t r = static_cast<uint8_t>((uint32_t)c.r * combined / (255u * 255u));
  const uint8_t g = static_cast<uint8_t>((uint32_t)c.g * combined / (255u * 255u));
  const uint8_t bl = static_cast<uint8_t>((uint32_t)c.b * combined / (255u * 255u));
  s->setPixelColor(0, s->Color(r, g, bl));
  s->show();
}

uint8_t StatusLed::buildColors(const DeviceStatus &st, Rgb out[kMaxColors]) const
{
  uint8_t n = 0;
  if (st.netConnected)
    out[n++] = {0, 255, 0}; // green: network up
  if (st.commConnecting)
    out[n++] = {0, 60, 255}; // blue: car link connecting / not connected
  if (st.error)
    out[n++] = {255, 0, 0}; // red: no net / upload error
  if (st.noGpsFix)
    out[n++] = {255, 160, 0}; // yellow/amber: no GPS fix
  return n;
}

void StatusLed::renderOnce()
{
  if (!started)
  {
    return;
  }

  DeviceStatus st;
  bool en;
  uint8_t b;
  portENTER_CRITICAL(&mux);
  st = status;
  en = enabled;
  b = bright;
  portEXIT_CRITICAL(&mux);

  if (!en || b == 0)
  {
    show({0, 0, 0}, 0);
    breathActive = false;
    return;
  }

  Rgb colors[kMaxColors];
  const uint8_t count = buildColors(st, colors);
  if (count == 0)
  {
    show({0, 0, 0}, 0); // nothing active -> off
    breathActive = false;
    return;
  }

  const uint32_t now = millis();

  // Hold one colour for a whole breath; only choose the next colour (and
  // re-sample the active set) when the current breath finishes, i.e. at its low
  // point. This way a status change that alters the colour set does not snap the
  // LED to a new colour mid-breath at high brightness — the new set is picked up
  // on the next breath, effectively queueing the change until the LED is near
  // off. buildColors() always lists conditions in the same fixed order, so the
  // cycle stays stable as conditions come and go.
  if (!breathActive || (now - breathStartMs) >= kBreathPeriodMs)
  {
    if (breathActive)
      breathIndex++; // advance to the next colour; fresh start keeps index as-is
    breathColor = colors[breathIndex % count];
    breathStartMs = now;
    breathActive = true;
  }

  // Symmetric triangle breath for the committed colour: fade up and down take
  // equal time, from ~12% to 100% (close to off at the ends, never fully dark).
  const uint32_t pos = now - breathStartMs; // 0..period
  const uint32_t half = kBreathPeriodMs / 2;
  const uint32_t up = (pos < half) ? pos : (kBreathPeriodMs - pos); // 0..half..0
  const uint8_t scale = static_cast<uint8_t>(30 + (uint32_t)(255 - 30) * up / half);

  show(breathColor, scale);
}
