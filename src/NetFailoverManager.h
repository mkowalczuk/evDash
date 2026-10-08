#pragma once

#include <Arduino.h>
#include <functional>

/**
 * Board-agnostic preferred/fallback network failover manager.
 *
 * Keeps a "preferred" link active and switches to a "fallback" link when the
 * preferred one stops passing traffic, then switches back when it recovers.
 * It is deliberately decoupled from any concrete transport (WiFi, cellular):
 * the owner wires up a small set of capability callbacks, so the same manager
 * can be reused on any board.
 *
 * Health is judged by a lightweight blocking probe (e.g. an ICMP ping to a
 * stable anchor like 1.1.1.1) that the owner supplies. Probes are throttled to
 * a configurable interval and never run every loop. All callbacks run from
 * loop(); none are called concurrently.
 *
 * Typical mapping (evDash transport policies):
 *   policy 0 (WiFi preferred, cellular fallback): preferred=WiFi, fallback=cell
 *   policy 1 (cellular preferred, WiFi fallback): preferred=cell, fallback=WiFi
 *   policies 2/3 are single-link and should not use this manager.
 */
class NetFailoverManager
{
public:
  enum class Link : uint8_t
  {
    Preferred,
    Fallback
  };

  struct Config
  {
    uint32_t probeIntervalMs = 60000;         // how often to probe the active link
    uint32_t fallbackRecheckStartMs = 60000;  // first preferred-recheck delay in fallback
    uint32_t fallbackRecheckMaxMs = 300000;   // backoff cap for preferred recheck
    uint8_t failureThreshold = 2;             // consecutive probe failures before switching
  };

  // Capability callbacks the owner provides. Probes and readiness are required;
  // bring-up / advance / log are optional (treated as no-ops / no-op success).
  struct Hooks
  {
    std::function<bool()> preferredReady;   // link-layer readiness of preferred
    std::function<bool()> fallbackReady;    // link-layer readiness of fallback
    std::function<bool()> probePreferred;   // connectivity probe (e.g. ping) preferred
    std::function<bool()> probeFallback;    // connectivity probe fallback
    std::function<void()> bringUpPreferred; // optional
    std::function<void()> bringUpFallback;  // optional
    std::function<bool()> advanceFallback;  // optional: next fallback option (e.g. SSID)
    // Called when the preferred link's probe fails, before/while deciding to
    // fail over. Lets a multi-option preferred link (e.g. several WiFi SSIDs)
    // rotate to its next option so it can recover without permanently living on
    // the fallback. Returns true if it advanced to a new option. Optional.
    std::function<bool()> onPreferredRetry;
    // Called once after the preferred link, having just been switched back to,
    // passes a confirming probe while active. Lets the owner tear down fallback
    // resources (e.g. close cellular data) only after the preferred link has
    // actually proven it works. Optional.
    std::function<void()> onPreferredConfirmed;
    std::function<void(const String &)> log;// optional
  };

  void begin(const Config &cfg, const Hooks &h)
  {
    config = cfg;
    hooks = h;
    active = Link::Preferred;
    consecutiveFailures = 0;
    lastProbeMs = 0;
    recheckBackoffMs = cfg.fallbackRecheckStartMs;
    pendingPreferredConfirm = false;
    started = true;
  }

  Link activeLink() const { return active; }
  bool onFallback() const { return active == Link::Fallback; }

  // Drive the state machine. Non-blocking except for the (throttled) probe call.
  void loop();

private:
  void logMsg(const String &s)
  {
    if (hooks.log)
    {
      hooks.log(s);
    }
  }
  bool call(const std::function<bool()> &fn, bool dflt) { return fn ? fn() : dflt; }
  void call(const std::function<void()> &fn) { if (fn) fn(); }

  Config config;
  Hooks hooks;
  bool started = false;
  Link active = Link::Preferred;
  uint8_t consecutiveFailures = 0;
  uint32_t lastProbeMs = 0;
  uint32_t recheckBackoffMs = 60000;
  // True after switching back to preferred, until a confirming probe passes.
  bool pendingPreferredConfirm = false;
};
