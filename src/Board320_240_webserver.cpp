// Display-only: draws on a 320x240 panel and reads touch and buttons.
// HTTP route handlers and the non-blocking server lifecycle live in
// BoardCore_webserver.cpp so headless boards can also use them.

#include "Board320_240.h"
#include <WiFi.h>
#include <WebServer.h>

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

      spr.fillRoundRect(cardX, btnY, btn1W, btnH, 6, 0x1A7D);
      spr.drawRoundRect(cardX, btnY, btn1W, btnH, 6, TFT_SKYBLUE);
      spr.setTextColor(TFT_WHITE, 0x1A7D);
      spr.setTextDatum(MC_DATUM);
      sprSetFont(fontRobotoThin24);
      sprDrawString("NEW PASS", cardX + btn1W / 2, btnY + btnH / 2);

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
  if (!startWebLogServer())
  {
    displayMessage("Web Log Server", "SD card not mounted");
    delay(2000);
    return;
  }

  const bool isSta    = !webLogServerSoftApStarted;
  const String ssid   = isSta ? WiFi.SSID()               : String("evDash");
  const String ip     = isSta ? WiFi.localIP().toString()  : WiFi.softAPIP().toString();

  drawWebLogServerScreen(ssid, ip, isSta);

  modalDialogActive    = true;
  messageDialogVisible = false;
  unsigned long startMs = millis();

  while (webLogServer != nullptr)  // stopWebLogServer() sets it to nullptr (idle timeout or touch)
  {
    webLogServerLoop();
    boardLoop();

    if (millis() - startMs > 500)
    {
      int16_t tx = 0, ty = 0;
      if (getTouch(tx, ty))
      {
        if (!isSta)
        {
          // "NEW PASS" button (x: 10..152, y >= 190)
          if (tx >= 10 && tx <= 152 && ty >= 190)
          {
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
            drawWebLogServerScreen(ssid, WiFi.softAPIP().toString(), false);
            startMs = millis();
            continue;
          }
          // "STOP & EXIT" button (x >= 158, y >= 190) or upper-left corner
          else if ((tx >= 158 && ty >= 190) || isUpperLeftTouch(tx, ty))
          {
            syslog->println("Web log server exit requested by touch");
            break;
          }
        }
        else
        {
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
  stopWebLogServer();
  displayMessage("Web Log Server", "Server stopped");
  delay(800);
}
