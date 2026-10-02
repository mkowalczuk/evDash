
// This file is display-only: it draws on a 320x240 panel and reads touch and
// buttons. A headless board has no screen, so it is not compiled at all there.
// Everything display-independent lives in BoardCore.cpp.
#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)

/**
 * evDash Web Log Server
 *
 * Provides a lightweight on-demand HTTP server to view, download, and manage
 * console logs (/logs/*.log) and driving logs (/*.json) directly from a PC or
 * phone browser over local WiFi or standalone Access Point mode.
 *
 * Runs strictly on-demand while the user remains in this screen and shuts down
 * completely upon exit for security, privacy, and low power.
 */

#include "Board320_240.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <FS.h>

static String formatBytes(uint64_t bytes)
{
  if (bytes < 1024)
  {
    return String((uint32_t)bytes) + " B";
  }
  else if (bytes < (1024ULL * 1024ULL))
  {
    return String((float)bytes / 1024.0f, 1) + " KB";
  }
  else if (bytes < (1024ULL * 1024ULL * 1024ULL))
  {
    return String((float)bytes / (1024.0f * 1024.0f), 2) + " MB";
  }
  else
  {
    return String((float)bytes / (1024.0f * 1024.0f * 1024.0f), 2) + " GB";
  }
}

static bool isSafeLogPath(const String &path)
{
  if (path.length() == 0 || path.indexOf("..") != -1)
  {
    return false;
  }
  if (!path.startsWith("/"))
  {
    return false;
  }
  if (path.endsWith(".log") || path.endsWith(".json") || path.endsWith(".bin"))
  {
    return true;
  }
  return false;
}

void Board320_240::handleWebLogRoot(WebServer &server)
{
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
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

  String statsStr = "<strong>" + formatBytes(usedBytes) + "</strong> used / " +
                    "<strong>" + formatBytes(totalBytes) + "</strong> total (" +
                    "<strong>" + formatBytes(freeBytes) + "</strong> free)</div>";
  server.sendContent(statsStr);

  // Section 1: Console logs in /logs
  server.sendContent(F("<h2>Console Logs (/logs)</h2>"
                       "<table><tr><th>Filename</th><th>Size</th><th>Actions</th></tr>"));

  bool foundConsole = false;
  File logsDir = SD.open("/logs");
  if (logsDir && logsDir.isDirectory())
  {
    while (true)
    {
      File entry = logsDir.openNextFile(FILE_READ);
      if (!entry)
      {
        break;
      }
      if (!entry.isDirectory())
      {
        String name = String(entry.name());
        if (name.endsWith(".log"))
        {
          foundConsole = true;
          String fullPath = name;
          if (!fullPath.startsWith("/logs/"))
          {
            if (fullPath.startsWith("/"))
              fullPath = "/logs" + fullPath;
            else
              fullPath = "/logs/" + fullPath;
          }
          String filenameOnly = fullPath.substring(fullPath.lastIndexOf('/') + 1);
          size_t sz = entry.size();

          String row = "<tr><td><strong>" + filenameOnly + "</strong></td><td>" + formatBytes(sz) + "</td><td>";
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
  {
    server.sendContent(F("<tr><td colspan='3' class='empty'>No console logs found in /logs</td></tr>"));
  }
  server.sendContent(F("</table>"));

  // Section 2: Driving and telemetry logs in root /
  server.sendContent(F("<h2>Driving & Telemetry Logs (/)</h2>"
                       "<table><tr><th>Filename</th><th>Size</th><th>Actions</th></tr>"));

  bool foundDriving = false;
  File rootDir = SD.open("/");
  if (rootDir && rootDir.isDirectory())
  {
    while (true)
    {
      File entry = rootDir.openNextFile(FILE_READ);
      if (!entry)
      {
        break;
      }
      if (!entry.isDirectory())
      {
        String name = String(entry.name());
        if (name.endsWith(".json"))
        {
          foundDriving = true;
          String fullPath = name;
          if (!fullPath.startsWith("/"))
          {
            fullPath = "/" + fullPath;
          }
          String filenameOnly = fullPath.substring(fullPath.lastIndexOf('/') + 1);
          size_t sz = entry.size();

          String row = "<tr><td><strong>" + filenameOnly + "</strong></td><td>" + formatBytes(sz) + "</td><td>";
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
  {
    server.sendContent(F("<tr><td colspan='3' class='empty'>No driving log files (.json) found</td></tr>"));
  }
  server.sendContent(F("</table>"));

  server.sendContent(F("<div class='footer'>Server running on evDash. Tap screen or press button on device to stop.</div>"
                       "</div></body></html>"));
  server.sendContent("");
}

void Board320_240::handleWebLogDownload(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  String path = server.arg("file");
  if (!isSafeLogPath(path) || !SD.exists(path))
  {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File f = SD.open(path, FILE_READ);
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

void Board320_240::handleWebLogView(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  String path = server.arg("file");
  if (!isSafeLogPath(path) || !SD.exists(path))
  {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File f = SD.open(path, FILE_READ);
  if (!f)
  {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  server.streamFile(f, "text/plain");
  f.close();
}

void Board320_240::handleWebLogDelete(WebServer &server)
{
  if (!server.hasArg("file"))
  {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  String path = server.arg("file");
  if (!isSafeLogPath(path) || !SD.exists(path))
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

  if (liveData->params.sdcardRecording && strlen(liveData->params.sdcardFilename) > 0)
  {
    if (path == liveData->params.sdcardFilename)
    {
      server.send(400, "text/plain", "Cannot delete currently active driving log");
      return;
    }
  }

  if (SD.remove(path.c_str()))
  {
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "Deleted");
  }
  else
  {
    server.send(500, "text/plain", "Failed to delete file");
  }
}

void Board320_240::drawWebLogServerScreen(const String &ssid, const String &ip, bool isSta)
{
  const uint16_t width = 320;
  const uint16_t height = 240;

  if (liveData->params.spriteInit)
  {
    spr.fillScreen(TFT_BLACK);

    // Top banner
    spr.fillRect(0, 0, width, 32, TFT_DARKGREEN);
    spr.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    spr.setTextDatum(MC_DATUM);
    sprSetFont(fontRobotoThin24);
    sprDrawString("WEB LOG SERVER", width / 2, 16);

    // Card background
    const int16_t cardX = 10, cardY = 40, cardW = 300, cardH = 152;
    spr.fillRoundRect(cardX, cardY, cardW, cardH, 8, 0x18C3);
    spr.drawRoundRect(cardX, cardY, cardW, cardH, 8, TFT_DARKGREY);

    spr.setTextDatum(TL_DATUM);
    sprSetFont(fontFont2);

    spr.setTextColor(TFT_SKYBLUE, 0x18C3);
    String wifiLine = isSta ? ("WiFi: " + ssid) : ("WiFi AP: " + ssid);
    sprDrawString(wifiLine.c_str(), cardX + 14, cardY + 14);

    spr.setTextColor(TFT_WHITE, 0x18C3);
    sprDrawString("Open in your browser:", cardX + 14, cardY + 40);

    spr.setTextColor(TFT_GREENYELLOW, 0x18C3);
    String urlLine = "http://" + ip + "/";
    sprDrawString(urlLine.c_str(), cardX + 14, cardY + 66);

    if (isSta)
    {
      spr.setTextColor(TFT_ORANGE, 0x18C3);
      sprDrawString("or  http://evdash.local/", cardX + 14, cardY + 92);
    }
    else
    {
      spr.setTextColor(TFT_ORANGE, 0x18C3);
      String passLine = "AP Pass: " + String(liveData->settings.webLogServerPassword);
      sprDrawString(passLine.c_str(), cardX + 14, cardY + 92);
    }

    spr.setTextColor(TFT_DARKGREY, 0x18C3);
    sprDrawString("Serving HTTP on port 80", cardX + 14, cardY + 120);

    const int16_t btnY = 200, btnH = 34;
    if (isSta)
    {
      spr.fillRoundRect(cardX, btnY, cardW, btnH, 6, TFT_DARKRED);
      spr.drawRoundRect(cardX, btnY, cardW, btnH, 6, TFT_RED);
      spr.setTextColor(TFT_WHITE, TFT_DARKRED);
      spr.setTextDatum(MC_DATUM);
      sprSetFont(fontRobotoThin24);
      sprDrawString("TAP TO STOP & EXIT", width / 2, btnY + btnH / 2);
    }
    else
    {
      const int16_t btn1W = 142;
      const int16_t btn2X = cardX + btn1W + 6;
      const int16_t btn2W = cardW - btn1W - 6;

      // Button 1: NEW PASS
      spr.fillRoundRect(cardX, btnY, btn1W, btnH, 6, 0x1A7D);
      spr.drawRoundRect(cardX, btnY, btn1W, btnH, 6, TFT_SKYBLUE);
      spr.setTextColor(TFT_WHITE, 0x1A7D);
      spr.setTextDatum(MC_DATUM);
      sprSetFont(fontRobotoThin24);
      sprDrawString("NEW PASS", cardX + btn1W / 2, btnY + btnH / 2);

      // Button 2: STOP & EXIT
      spr.fillRoundRect(btn2X, btnY, btn2W, btnH, 6, TFT_DARKRED);
      spr.drawRoundRect(btn2X, btnY, btn2W, btnH, 6, TFT_RED);
      spr.setTextColor(TFT_WHITE, TFT_DARKRED);
      sprDrawString("STOP & EXIT", btn2X + btn2W / 2, btnY + btnH / 2);
    }

    spr.pushSprite(0, 0);
  }
  else
  {
    tft.fillScreen(TFT_BLACK);
    tft.fillRect(0, 0, width, 32, TFT_DARKGREEN);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    tft.setTextDatum(MC_DATUM);
    tft.setFont(fontRobotoThin24);
    tft.drawString("WEB LOG SERVER", width / 2, 16);

    const int16_t cardX = 10, cardY = 40, cardW = 300, cardH = 152;
    tft.fillRoundRect(cardX, cardY, cardW, cardH, 8, 0x18C3);
    tft.drawRoundRect(cardX, cardY, cardW, cardH, 8, TFT_DARKGREY);

    tft.setTextDatum(TL_DATUM);
    tft.setFont(fontFont2);

    tft.setTextColor(TFT_SKYBLUE, 0x18C3);
    String wifiLine = isSta ? ("WiFi: " + ssid) : ("WiFi AP: " + ssid);
    tft.drawString(wifiLine.c_str(), cardX + 14, cardY + 14);

    tft.setTextColor(TFT_WHITE, 0x18C3);
    tft.drawString("Open in your browser:", cardX + 14, cardY + 40);

    tft.setTextColor(TFT_GREENYELLOW, 0x18C3);
    String urlLine = "http://" + ip + "/";
    tft.drawString(urlLine.c_str(), cardX + 14, cardY + 66);

    if (isSta)
    {
      tft.setTextColor(TFT_ORANGE, 0x18C3);
      tft.drawString("or  http://evdash.local/", cardX + 14, cardY + 92);
    }
    else
    {
      tft.setTextColor(TFT_ORANGE, 0x18C3);
      String passLine = "AP Pass: " + String(liveData->settings.webLogServerPassword);
      tft.drawString(passLine.c_str(), cardX + 14, cardY + 92);
    }

    tft.setTextColor(TFT_DARKGREY, 0x18C3);
    tft.drawString("Serving HTTP on port 80", cardX + 14, cardY + 120);

    const int16_t btnY = 200, btnH = 34;
    if (isSta)
    {
      tft.fillRoundRect(cardX, btnY, cardW, btnH, 6, TFT_DARKRED);
      tft.drawRoundRect(cardX, btnY, cardW, btnH, 6, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_DARKRED);
      tft.setTextDatum(MC_DATUM);
      tft.setFont(fontRobotoThin24);
      tft.drawString("TAP TO STOP & EXIT", width / 2, btnY + btnH / 2);
    }
    else
    {
      const int16_t btn1W = 142;
      const int16_t btn2X = cardX + btn1W + 6;
      const int16_t btn2W = cardW - btn1W - 6;

      tft.fillRoundRect(cardX, btnY, btn1W, btnH, 6, 0x1A7D);
      tft.drawRoundRect(cardX, btnY, btn1W, btnH, 6, TFT_SKYBLUE);
      tft.setTextColor(TFT_WHITE, 0x1A7D);
      tft.setTextDatum(MC_DATUM);
      tft.setFont(fontRobotoThin24);
      tft.drawString("NEW PASS", cardX + btn1W / 2, btnY + btnH / 2);

      tft.fillRoundRect(btn2X, btnY, btn2W, btnH, 6, TFT_DARKRED);
      tft.drawRoundRect(btn2X, btnY, btn2W, btnH, 6, TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_DARKRED);
      tft.drawString("STOP & EXIT", btn2X + btn2W / 2, btnY + btnH / 2);
    }
  }
}

void Board320_240::runWebLogServer()
{
  if (!liveData->settings.sdcardEnabled || !sdcardMount())
  {
    displayMessage("Web Log Server", "SD card not mounted");
    delay(2000);
    return;
  }

  if (!isValidPassword(liveData->settings.webLogServerPassword, 8))
  {
    generateRandomAlphanumeric(liveData->settings.webLogServerPassword, 8);
    saveSettings();
  }

  const bool wasConnected = (WiFi.status() == WL_CONNECTED);
  bool startedSoftAp = false;
  String ipStr = "";
  String ssidStr = "";

  if (wasConnected)
  {
    ssidStr = WiFi.SSID();
    ipStr = WiFi.localIP().toString();
  }
  else
  {
    displayMessage("Web Log Server", "Starting Access Point...");
    WiFi.disconnect(true);
    delay(50);
    WiFi.mode(WIFI_AP);
    IPAddress apIP(10, 0, 0, 1);
    IPAddress gateway(10, 0, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dhcpStart(10, 0, 0, 2);
    WiFi.softAPConfig(apIP, gateway, subnet, dhcpStart);
    startedSoftAp = WiFi.softAP("evDash", liveData->settings.webLogServerPassword, 1, 0, 4);
    ssidStr = "evDash";
    ipStr = WiFi.softAPIP().toString();
    syslog->printf("WiFi AP evDash started: IP %s, SSID %s, pass %s\n", ipStr.c_str(), ssidStr.c_str(), liveData->settings.webLogServerPassword);
  }

  bool mdnsStarted = MDNS.begin("evdash");
  if (mdnsStarted)
  {
    MDNS.addService("http", "tcp", 80);
  }

  WebServer server(80);
  server.on("/", HTTP_GET, [this, &server]() {
    handleWebLogRoot(server);
  });
  server.on("/view", HTTP_GET, [this, &server]() {
    handleWebLogView(server);
  });
  server.on("/download", HTTP_GET, [this, &server]() {
    handleWebLogDownload(server);
  });
  server.on("/delete", HTTP_POST, [this, &server]() {
    handleWebLogDelete(server);
  });
  server.onNotFound([&server]() {
    server.send(404, "text/plain", "Not Found");
  });

  server.begin();
  syslog->printf("Web log server started at http://%s/\n", ipStr.c_str());

  drawWebLogServerScreen(ssidStr, ipStr, wasConnected);

  modalDialogActive = true;
  messageDialogVisible = false;
  unsigned long startMs = millis();

  while (true)
  {
    server.handleClient();
    boardLoop();

    // Debounce touch/buttons for 500ms after starting
    if (millis() - startMs > 500)
    {
      int16_t tx = 0, ty = 0;
      if (getTouch(tx, ty))
      {
        if (startedSoftAp)
        {
          // Check if "NEW PASS" button was tapped (x: 10..152, y >= 190)
          if (tx >= 10 && tx <= 152 && ty >= 190)
          {
            syslog->println("Regenerating AP password...");
            generateRandomAlphanumeric(liveData->settings.webLogServerPassword, 8);
            saveSettings();

            WiFi.softAPdisconnect(true);
            delay(50);
            WiFi.mode(WIFI_AP);
            IPAddress apIP(10, 0, 0, 1);
            IPAddress gateway(10, 0, 0, 1);
            IPAddress subnet(255, 255, 255, 0);
            IPAddress dhcpStart(10, 0, 0, 2);
            WiFi.softAPConfig(apIP, gateway, subnet, dhcpStart);
            WiFi.softAP("evDash", liveData->settings.webLogServerPassword, 1, 0, 4);

            syslog->printf("New AP password: %s\n", liveData->settings.webLogServerPassword);
            drawWebLogServerScreen(ssidStr, ipStr, wasConnected);
            startMs = millis();
            continue;
          }
          // Check if "STOP & EXIT" button was tapped (x >= 158, y >= 190) or upper-left corner
          else if ((tx >= 158 && ty >= 190) || isUpperLeftTouch(tx, ty))
          {
            syslog->println("Web log server exit requested by touch");
            break;
          }
        }
        else
        {
          // In STA mode, any touch exits
          syslog->println("Web log server exit requested by touch");
          break;
        }
      }

      if (isButtonPressed(pinButtonLeft) || isButtonPressed(pinButtonRight) || isButtonPressed(pinButtonMiddle))
      {
        syslog->println("Web log server exit requested by button");
        break;
      }
    }
    delay(5);
  }

  modalDialogActive = false;
  server.stop();

  if (mdnsStarted)
  {
    MDNS.end();
  }

  if (startedSoftAp)
  {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    if (liveData->settings.wifiEnabled == 1)
    {
      wifiSetup();
    }
  }

  displayMessage("Web Log Server", "Server stopped");
  delay(800);
}
#endif // BOARD_M5STACK_CORE2 || BOARD_M5STACK_CORES3
