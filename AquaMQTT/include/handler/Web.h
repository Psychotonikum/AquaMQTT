#ifndef AQUAMQTT_WEB_H
#define AQUAMQTT_WEB_H

#include <ESPAsyncWebServer.h>

namespace aquamqtt
{

class WebHandler
{
public:
    WebHandler();
    void setup();
    void loop();
    static bool saveConfigFile(const String& filename, const String& content);

private:
    static void handleRoot(AsyncWebServerRequest* request);
    static void handleNotFound(AsyncWebServerRequest* request);
    static void handleWifiGet(AsyncWebServerRequest* request);
    static void handleMqttGet(AsyncWebServerRequest* request);
    static void handleAquaGet(AsyncWebServerRequest* request);
    static void handleReboot(AsyncWebServerRequest* request);

    static AsyncWebServer mServer;
};

}  // namespace aquamqtt

#endif  // AQUAMQTT_WEB_H
