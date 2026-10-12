#include "NetFailoverManager.h"

/**
 * Failover state machine. Called every board loop; it rate-limits its own work
 * so the actual (blocking) probe runs at most once per interval.
 *
 * On the PREFERRED link:
 *   - if the link is down at L2 (preferredReady false, e.g. WiFi not
 *     associated), fail over immediately: try rotating to another preferred
 *     option (e.g. next WiFi SSID) first, otherwise switch to the fallback at
 *     once without waiting a probe interval or a second failure.
 *   - if the link is up, probe every probeIntervalMs; on failureThreshold
 *     consecutive probe failures, switch to the fallback (bringing it up first)
 *     and start the recheck timer.
 *
 * On the FALLBACK link:
 *   - periodically (recheck backoff, growing to the cap) re-probe the PREFERRED
 *     link; when it passes, switch back.
 *   - also probe the fallback itself; if the fallback is ALSO failing, there is
 *     nothing to lose, so retry the preferred immediately (and advance to the
 *     next fallback option, e.g. the next WiFi SSID, if one is available).
 */
void NetFailoverManager::loop()
{
  if (!started)
  {
    return;
  }

  const uint32_t now = millis();

  if (active == Link::Preferred)
  {
    // Link-layer down (e.g. WiFi not associated / no AP found) is immediate and
    // definitive: there is nothing to ping and waiting a full probe interval or
    // a second failure just strands us without connectivity. Fail over at once,
    // bypassing both the probe throttle and the failure-count threshold. The
    // probe/threshold machinery below is only for damping a *connected* link
    // that drops the occasional packet.
    if (!call(hooks.preferredReady, true))
    {
      // Throttle this path so a freshly-selected SSID gets a few seconds to
      // associate before we judge it down and rotate again. Much shorter than
      // probeIntervalMs (which paces a *connected* link's reachability probe),
      // so failover to cellular still completes in seconds, not a minute.
      if ((now - lastProbeMs) < kL2AssociateWaitMs)
      {
        return;
      }
      lastProbeMs = now;

      // Let a multi-option preferred link (e.g. several WiFi SSIDs) try its next
      // option first; if that associates, we stay on preferred and re-probe.
      // Bound the rotation so a set of all-absent SSIDs can't spin here forever:
      // after a full sweep's worth of attempts we fail over regardless.
      if (l2DownRotations < kMaxL2DownRotations && call(hooks.onPreferredRetry, false))
      {
        l2DownRotations++;
        consecutiveFailures = 0;
        return;
      }
      logMsg("NET: preferred link not associated, switching to fallback immediately");
      call(hooks.bringUpFallback);
      active = Link::Fallback;
      consecutiveFailures = 0;
      l2DownRotations = 0;
      recheckBackoffMs = config.fallbackRecheckStartMs;
      lastProbeMs = now;
      return;
    }
    l2DownRotations = 0;

    if ((now - lastProbeMs) < config.probeIntervalMs)
    {
      return;
    }
    lastProbeMs = now;

    // Link is up at L2; judge reachability by the connectivity probe.
    const bool ok = call(hooks.probePreferred, false);
    if (ok)
    {
      consecutiveFailures = 0;
      // If we recently switched back to preferred, this successful probe is the
      // confirmation that it actually works. Only now tear down fallback (e.g.
      // close cellular data), so we never drop the fallback before the preferred
      // link has proven itself.
      if (pendingPreferredConfirm)
      {
        pendingPreferredConfirm = false;
        logMsg("NET: preferred link confirmed working, releasing fallback");
        call(hooks.onPreferredConfirmed);
      }
      return;
    }

    if (++consecutiveFailures < config.failureThreshold)
    {
      logMsg(String("NET: preferred link probe failed (") + consecutiveFailures + "/" +
             config.failureThreshold + ")");
      // Give a multi-option preferred link (e.g. WiFi SSIDs) a chance to rotate
      // to another option before we give up and fail over.
      call(hooks.onPreferredRetry, false);
      return;
    }

    logMsg("NET: preferred link down, switching to fallback");
    // Rotate the preferred option once more so that, while on fallback, the
    // periodic preferred-recheck tries a different SSID rather than the one that
    // just failed.
    call(hooks.onPreferredRetry, false);
    call(hooks.bringUpFallback);
    active = Link::Fallback;
    consecutiveFailures = 0;
    recheckBackoffMs = config.fallbackRecheckStartMs;
    lastProbeMs = now;
    return;
  }

  // active == Link::Fallback
  if ((now - lastProbeMs) < recheckBackoffMs)
  {
    return;
  }
  lastProbeMs = now;

  // Has the preferred link recovered?
  const bool preferredUp = call(hooks.preferredReady, true) && call(hooks.probePreferred, false);
  if (preferredUp)
  {
    logMsg("NET: preferred link recovered, switching back (fallback kept until confirmed)");
    call(hooks.bringUpPreferred);
    active = Link::Preferred;
    consecutiveFailures = 0;
    recheckBackoffMs = config.fallbackRecheckStartMs;
    // Do NOT tear down the fallback yet. Keep it until the preferred link passes
    // a confirming probe while active (handled in the preferred branch). This
    // avoids dropping the working fallback before the preferred truly works.
    pendingPreferredConfirm = true;
    lastProbeMs = 0; // confirm on the next loop, then release fallback promptly
    return;
  }

  // Preferred still down. Check whether the fallback itself is still working.
  const bool fallbackUp = call(hooks.fallbackReady, true) && call(hooks.probeFallback, false);
  if (!fallbackUp)
  {
    // Both links are failing: nothing to lose, so try to rotate the fallback
    // (e.g. next WiFi SSID) and retry the preferred immediately next loop.
    const bool advanced = call(hooks.advanceFallback, false);
    logMsg(advanced ? "NET: fallback also down, trying next fallback option"
                    : "NET: both links down, retrying preferred immediately");
    recheckBackoffMs = config.fallbackRecheckStartMs;
    lastProbeMs = 0; // force an immediate re-probe on the next loop
    return;
  }

  // Fallback healthy, preferred still down: back off the preferred recheck, and
  // rotate the preferred link's option (e.g. next WiFi SSID) so the next recheck
  // attempts a different one instead of retrying the same failed SSID forever.
  call(hooks.onPreferredRetry, false);
  recheckBackoffMs = min<uint32_t>(recheckBackoffMs * 2, config.fallbackRecheckMaxMs);
}
