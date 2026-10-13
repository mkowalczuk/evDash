/**
 * evDash Web Log Server — shared (board-neutral) implementation.
 *
 * HTTP route handlers and the non-blocking server lifecycle (start / loop /
 * stop) live here so both the M5 display boards and the headless Waveshare
 * board can use them.  Only the screen-drawing code stays in
 * Board320_240_webserver.cpp.
 *
 * The server is non-blocking: startWebLogServer() sets up WiFi/AP and calls
 * server.begin(), webLogServerLoop() calls server.handleClient() once per
 * netLoop() tick, and stopWebLogServer() tears everything down.  An idle
 * timeout (webServerTimeoutSec) auto-stops the server when no HTTP request
 * has been handled for that many seconds.
 *
 * boardFilesystem() is used for all file access so the SPI-SD (M5) and
 * SD_MMC (Waveshare) backends are both supported without #ifdef.
 */

#include "BoardCore.h"
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static String wls_formatBytes(uint64_t bytes)
{
  if (bytes < 1024ULL)
    return String((uint32_t)bytes) + " B";
  if (bytes < 1024ULL * 1024ULL)
    return String((float)bytes / 1024.0f, 1) + " KB";
  if (bytes < 1024ULL * 1024ULL * 1024ULL)
    return String((float)bytes / (1024.0f * 1024.0f), 2) + " MB";
  return String((float)bytes / (1024.0f * 1024.0f * 1024.0f), 2) + " GB";
}

static bool wls_isSafeLogPath(const String &path)
{
  if (path.length() == 0 || path.indexOf("..") != -1)
    return false;
  if (!path.startsWith("/"))
    return false;
  return path.endsWith(".log") || path.endsWith(".json") || path.endsWith(".bin");
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------

void BoardCore::handleWebLogRoot(WebServer &server)
{
  if (syslog != nullptr && syslog->isSdLogging())
    syslog->flushSdBuffer(0, true);

  webLogServerLastActivityMs = millis();

  fs::FS &fs = boardFilesystem();

  // Storage usage via the board's concrete FS (SPI SD or SD_MMC).
  uint64_t totalBytes = 0, usedBytes = 0;
  boardFilesystemBytes(totalBytes, usedBytes);
  uint64_t freeBytes = (totalBytes > usedBytes) ? (totalBytes - usedBytes) : 0;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  server.sendContent(F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                       "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                       "<title>evDash Log Manager</title><style>"
                       "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:20px;}"
                       ".container{max-width:960px;margin:0 auto;}"
                       "h1{color:#38bdf8;margin:0 0 6px 0;font-size:1.6rem;}"
                       ".subtitle{color:#94a3b8;font-size:0.9rem;margin-bottom:16px;}"
                       ".stats{background:#1e293b;padding:12px 18px;border-radius:8px;margin-bottom:24px;font-size:0.95rem;color:#94a3b8;border:1px solid #334155;}"
                       ".stats strong{color:#38bdf8;}"
                       "h2{color:#f1f5f9;font-size:1.15rem;margin:24px 0 10px 0;border-bottom:1px solid #334155;padding-bottom:6px;}"
                       "table{width:100%;border-collapse:collapse;margin-bottom:24px;background:#1e293b;border-radius:8px;overflow:hidden;border:1px solid #334155;}"
                       "th,td{padding:10px 14px;text-align:left;}"
                       "th{background:#0b1120;color:#94a3b8;font-weight:600;font-size:0.8rem;text-transform:uppercase;letter-spacing:0.05em;}"
                       "tr:not(:last-child){border-bottom:1px solid #334155;}"
                       "tr:hover{background:#283548;}"
                       ".btn{display:inline-block;padding:5px 12px;border-radius:4px;text-decoration:none;font-size:0.85rem;font-weight:500;margin-right:6px;}"
                       ".btn-view{background:#0284c7;color:#fff;}"
                       ".btn-dl{background:#16a34a;color:#fff;}"
                       ".btn-del{background:#dc2626;color:#fff;border:none;cursor:pointer;padding:6px 12px;border-radius:4px;font-size:0.85rem;}"
                       ".btn:hover{opacity:0.88;}"
                       ".empty{color:#64748b;font-style:italic;padding:16px;}"
                       ".footer{text-align:center;color:#64748b;margin-top:36px;font-size:0.85rem;padding:12px;border-top:1px solid #1e293b;}"
                       "</style></head><body><div class='container'>"
                       "<h1>evDash Log Manager</h1>"
                       "<div class='subtitle'>Direct storage access over local WiFi</div>"
                       "<div class='stats'>Storage: "));

  String statsStr = "<strong>" + wls_formatBytes(usedBytes) + "</strong> used / " +
                    "<strong>" + wls_formatBytes(totalBytes) + "</strong> total (" +
                    "<strong>" + wls_formatBytes(freeBytes) + "</strong> free)</div>";
  server.sendContent(statsStr);

  // Section 1: Console logs in /logs
  server.sendContent(F("<h2>Console Logs (/logs)</h2>"
                       "<table><tr><th>Filename</th><th>Size</th><th>Actions</th></tr>"));

  bool foundConsole = false;
  File logsDir = fs.open("/logs");
  if (logsDir && logsDir.isDirectory())
  {
    while (true)
    {
      File entry = logsDir.openNextFile();
      if (!entry) break;
      if (!entry.isDirectory())
      {
        String name = String(entry.name());
        if (name.endsWith(".log"))
        {
          foundConsole = true;
          String fullPath = name;
          if (!fullPath.startsWith("/logs/"))
            fullPath = fullPath.startsWith("/") ? ("/logs" + fullPath) : ("/logs/" + fullPath);
          String filenameOnly = fullPath.substring(fullPath.lastIndexOf('/') + 1);
          size_t sz = entry.size();

          String row = "<tr><td><strong>" + filenameOnly + "</strong></td><td>" + wls_formatBytes(sz) + "</td><td>";
          row += "<a class='btn btn-view' href='/view?file=" + fullPath + "' target='_blank'>View</a>";
          row += "<a class='btn btn-dl' href='/download?file=" + fullPath + "'>Download</a>";
          row += "<form method='POST' action='/delete' style='display:inline;' onsubmit=\"return confirm('Delete " + filenameOnly + "?');\">";
          row += "<input type='hidden' name='file' value='" + fullPath + "'>";
          row += "<button type='submit' class='btn-del'>Delete</button></form>";
          row += "</td></tr>";
          server.sendContent(row);
        }
      }
      entry.close();
    }
    logsDir.close();
  }
  if (!foundConsole)
    server.sendContent(F("<tr><td colspan='3' class='empty'>No console logs found in /logs</td></tr>"));
  server.sendContent(F("</table>"));

  // Section 2: Driving logs in root /
  server.sendContent(F("<h2>Driving &amp; Telemetry Logs (/)</h2>"
                       "<table><tr><th>Filename</th><th>Size</th><th>Actions</th></tr>"));

  bool foundDriving = false;
  File rootDir = fs.open("/");
  if (rootDir && rootDir.isDirectory())
  {
    while (true)
    {
      File entry = rootDir.openNextFile();
      if (!entry) break;
      if (!entry.isDirectory())
      {
        String name = String(entry.name());
        if (name.endsWith(".json"))
        {
          foundDriving = true;
          String fullPath = name.startsWith("/") ? name : ("/" + name);
          String filenameOnly = fullPath.substring(fullPath.lastIndexOf('/') + 1);
          size_t sz = entry.size();

          String row = "<tr><td><strong>" + filenameOnly + "</strong></td><td>" + wls_formatBytes(sz) + "</td><td>";
          row += "<a class='btn btn-view' href='/view?file=" + fullPath + "' target='_blank'>View</a>";
          row += "<a class='btn btn-dl' href='/download?file=" + fullPath + "'>Download</a>";
          row += "<form method='POST' action='/delete' style='display:inline;' onsubmit=\"return confirm('Delete " + filenameOnly + "?');\">";
          row += "<input type='hidden' name='file' value='" + fullPath + "'>";
          row += "<button type='submit' class='btn-del'>Delete</button></form>";
          row += "</td></tr>";
          server.sendContent(row);
        }
      }
      entry.close();
    }
    rootDir.close();
  }
  if (!foundDriving)
    server.sendContent(F("<tr><td colspan='3' class='empty'>No driving log files (.json) found</td></tr>"));
  server.sendContent(F("</table>"));

  server.sendContent(F("<div class='footer'>evDash web log server &mdash; use <code>webServer=0</code> in console to stop.</div>"
                       "</div></body></html>"));
  server.sendContent("");
}

void BoardCore::handleWebLogDownload(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  if (syslog != nullptr && syslog->isSdLogging())
    syslog->flushSdBuffer(0, true);

  webLogServerLastActivityMs = millis();

  fs::FS &fs = boardFilesystem();
  String path = server.arg("file");
  if (!wls_isSafeLogPath(path) || !fs.exists(path))
  {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File f = fs.open(path, FILE_READ);
  if (!f)
  {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  String filenameOnly = path.substring(path.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filenameOnly + "\"");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

void BoardCore::handleWebLogView(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }
  if (syslog != nullptr && syslog->isSdLogging())
    syslog->flushSdBuffer(0, true);

  webLogServerLastActivityMs = millis();

  fs::FS &fs = boardFilesystem();
  String path = server.arg("file");
  if (!wls_isSafeLogPath(path) || !fs.exists(path))
  {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File f = fs.open(path, FILE_READ);
  if (!f)
  {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }
  server.streamFile(f, "text/plain");
  f.close();
}

void BoardCore::handleWebLogDelete(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  webLogServerLastActivityMs = millis();

  fs::FS &fs = boardFilesystem();
  String path = server.arg("file");
  if (!wls_isSafeLogPath(path) || !fs.exists(path))
  {
    server.send(404, "text/plain", "File not found");
    return;
  }

  const char *activeConsoleLog = syslog->getSdLogPath();
  if (activeConsoleLog != nullptr && path == activeConsoleLog)
  {
    server.send(400, "text/plain", "Cannot delete currently active console log");
    return;
  }
  if (liveData->params.sdcardRecording && strlen(liveData->params.sdcardFilename) > 0 &&
      path == liveData->params.sdcardFilename)
  {
    server.send(400, "text/plain", "Cannot delete currently active driving log");
    return;
  }

  if (fs.remove(path.c_str()))
  {
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "Deleted");
  }
  else
  {
    server.send(500, "text/plain", "Failed to delete file");
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool BoardCore::startWebLogServer()
{
  if (webLogServer != nullptr)
  {
    syslog->println("Web log server already running.");
    return true;
  }
  if (!liveData->settings.sdcardEnabled || !sdcardMount())
  {
    syslog->println("Web log server: SD card not available.");
    return false;
  }
  if (!isValidPassword(liveData->settings.webLogServerPassword, 8))
  {
    generateRandomAlphanumeric(liveData->settings.webLogServerPassword, 8);
    saveSettings();
  }

  const bool wasConnected = (WiFi.status() == WL_CONNECTED);
  String ssidStr, ipStr;

  if (wasConnected)
  {
    ssidStr = WiFi.SSID();
    ipStr   = WiFi.localIP().toString();
    webLogServerSoftApStarted = false;
  }
  else
  {
    WiFi.disconnect(true);
    delay(50);
    WiFi.mode(WIFI_AP);
    IPAddress apIP(10, 0, 0, 1);
    IPAddress gateway(10, 0, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dhcpStart(10, 0, 0, 2);
    WiFi.softAPConfig(apIP, gateway, subnet, dhcpStart);
    webLogServerSoftApStarted = WiFi.softAP("evDash", liveData->settings.webLogServerPassword, 1, 0, 4);
    ssidStr = "evDash";
    ipStr   = WiFi.softAPIP().toString();
    syslog->printf("WiFi AP evDash started: IP %s, pass %s\n", ipStr.c_str(), liveData->settings.webLogServerPassword);
  }

  webLogServer = new WebServer(80);
  webLogServer->on("/", HTTP_GET, [this]() {
    handleWebLogRoot(*webLogServer);
  });
  webLogServer->on("/view", HTTP_GET, [this]() {
    handleWebLogView(*webLogServer);
  });
  webLogServer->on("/download", HTTP_GET, [this]() {
    handleWebLogDownload(*webLogServer);
  });
  webLogServer->on("/delete", HTTP_POST, [this]() {
    handleWebLogDelete(*webLogServer);
  });
  webLogServer->onNotFound([this]() {
    webLogServer->send(404, "text/plain", "Not Found");
  });

  webLogServer->begin();
  webLogServerLastActivityMs = millis();

  syslog->printf("Web log server started: http://%s/\n", ipStr.c_str());
  if (!wasConnected)
    syslog->printf("  WiFi AP: evDash  Pass: %s\n", liveData->settings.webLogServerPassword);

  const uint16_t tSec = liveData->settings.webServerTimeoutSec;
  if (tSec > 0)
    syslog->printf("  Auto-stop after %u sec of inactivity. Use webServer=0 to stop manually.\n", tSec);
  else
    syslog->println("  No idle timeout. Use webServer=0 to stop.");

  return true;
}

void BoardCore::stopWebLogServer()
{
  if (webLogServer == nullptr)
    return;

  webLogServer->stop();
  delete webLogServer;
  webLogServer = nullptr;

  if (webLogServerSoftApStarted)
  {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    webLogServerSoftApStarted = false;
    if (liveData->settings.wifiEnabled == 1)
      wifiSetup();
  }

  syslog->println("Web log server stopped.");
}

void BoardCore::webLogServerLoop()
{
  if (webLogServer == nullptr)
    return;

  webLogServer->handleClient();

  // Idle timeout
  const uint16_t tSec = liveData->settings.webServerTimeoutSec;
  if (tSec > 0)
  {
    const uint32_t idleMs = millis() - webLogServerLastActivityMs;
    if (idleMs >= static_cast<uint32_t>(tSec) * 1000U)
    {
      syslog->printf("Web log server: idle timeout (%u sec). Stopping.\n", tSec);
      stopWebLogServer();
    }
  }
}
