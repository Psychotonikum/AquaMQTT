#include "handler/Web.h"

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "config/WebConfig.h"
#include "state/Optitronic2State.h"
#include "task/ModbusRelayTask.h"
#include "task/Optitronic2MQTTTask.h"

extern aquamqtt::ModbusRelayTask modbusRelayTask;
extern aquamqtt::Optitronic2MQTTTask optitronic2MqttTask;

namespace aquamqtt
{

AsyncWebServer WebHandler::mServer(80);

WebHandler::WebHandler()
{
}

void WebHandler::setup()
{
    // Serve static files for React SPA
    mServer.serveStatic("/assets/", LittleFS, "/assets/");
    mServer.serveStatic("/css/", LittleFS, "/css/");
    mServer.serveStatic("/fonts/", LittleFS, "/fonts/");
    mServer.serveStatic("/app/", LittleFS, "/app/");
    mServer.serveStatic("/favicon.ico", LittleFS, "/favicon.ico");
    mServer.serveStatic("/web/", LittleFS, "/web/");

    // Root page
    mServer.on("/", HTTP_GET, handleRoot);

    // GET API endpoints — return stored config JSON
    mServer.on("/api/wifi", HTTP_GET, handleWifiGet);
    mServer.on("/api/mqtt", HTTP_GET, handleMqttGet);
    mServer.on("/api/aquamqtt", HTTP_GET, handleAquaGet);

    // Reboot endpoint
    mServer.on("/reboot", HTTP_GET, handleReboot);

    // Diagnostics endpoint
    mServer.on("/api/diag", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"framesReceived\":";
        json += modbusRelayTask.getFramesReceived();
        json += ",\"crcErrors\":";
        json += modbusRelayTask.getCrcErrors();
        json += ",\"framesRelayed\":";
        json += modbusRelayTask.getFramesRelayed();
        json += ",\"hmiFramesIn\":";
        json += modbusRelayTask.getHmiFramesIn();
        json += ",\"mainFramesIn\":";
        json += modbusRelayTask.getMainFramesIn();
        json += ",\"hmiBytesIn\":";
        json += modbusRelayTask.getHmiBytesIn();
        json += ",\"mainBytesIn\":";
        json += modbusRelayTask.getMainBytesIn();
        json += ",\"echoBytes\":";
        json += modbusRelayTask.getEchoBytes();
        json += ",\"txBytesWritten\":";
        json += modbusRelayTask.getTxBytesWritten();
        json += ",\"writesInjected\":";
        json += modbusRelayTask.getWritesInjected();
        json += ",\"lastFrame\":\"";
        for (int i = 0; i < modbusRelayTask.getLastFwdFrameLen(); i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", modbusRelayTask.getLastFwdFrame()[i]);
            json += hex;
        }
        json += "\"";
        json += ",\"uptime\":";
        json += millis();
        json += "}";

        request->send(200, "application/json", json);
    });

    // Register dump endpoint
    mServer.on("/api/registers", HTTP_GET, [](AsyncWebServerRequest* request) {
        auto& state = aquamqtt::Optitronic2State::getInstance();
        String json = "{";
        bool first = true;
        uint16_t val;
        for (uint16_t r = 0; r < 0x0321; r++)
        {
            if (state.getRegister(r, val))
            {
                if (!first) json += ",";
                char key[12];
                snprintf(key, sizeof(key), "\"0x%04X\"", r);
                json += key;
                json += ":";
                json += val;
                first = false;
            }
        }
        json += "}";
        request->send(200, "application/json", json);
    });

    // Structured status endpoint for web UI
    mServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        using namespace aquamqtt::message::optitronic2;
        auto& state = aquamqtt::Optitronic2State::getInstance();
        String json = "{";

        // Sensors
        json += "\"waterTemp\":"; json += String(state.getWaterTemp(), 1);
        json += ",\"ambientTemp\":"; json += String(state.getAmbientTemp(), 1);
        json += ",\"evaporatorTemp\":"; json += String(state.getEvaporatorTemp(), 1);

        // State
        json += ",\"operatingState\":"; json += state.getOperatingState();
        json += ",\"heatSource\":"; json += state.getHeatSource();
        json += ",\"activeSetpoint\":"; json += String(state.getActiveSetpoint(), 1);
        json += ",\"quickHeatActive\":"; json += state.isQuickHeatActive() ? "true" : "false";
        json += ",\"forceHeating\":"; json += state.isForceHeating() ? "true" : "false";
        json += ",\"pvActive\":"; json += state.isPvActive() ? "true" : "false";

        // Settings
        json += ",\"dhwSetpoint\":"; json += String(state.getDhwSetpoint(), 1);
        json += ",\"ecoDeviation\":"; json += String(state.getEcoDeviation(), 1);
        json += ",\"komfortDeviation\":"; json += String(state.getKomfortDeviation(), 1);
        json += ",\"program\":"; json += state.getProgram();
        json += ",\"auxHeatMode\":"; json += state.getAuxHeatMode();
        json += ",\"extInputFunction\":"; json += state.getExtInputFunction();

        // Installer
        if (state.hasBlock(REG_INSTALLER_BLK1_START))
        {
            json += ",\"frostProtectTemp\":"; json += String(state.getFrostProtectTemp(), 1);
            json += ",\"antiLegioInterval\":"; json += state.getAntiLegioInterval();
        }
        if (state.hasBlock(REG_INSTALLER_BLK2_START))
        {
            json += ",\"bivalentThreshold\":"; json += String(state.getBivalentThreshold(), 1);
            json += ",\"pvTargetSetpoint\":"; json += String(state.getPvTargetSetpoint(), 1);
            json += ",\"extSourceMaxTemp\":"; json += String(state.getExtSourceMaxTemp(), 1);
        }

        // Diag
        json += ",\"uptime\":"; json += millis();
        json += ",\"framesReceived\":"; json += modbusRelayTask.getFramesReceived();
        json += ",\"crcErrors\":"; json += modbusRelayTask.getCrcErrors();
        json += ",\"writesInjected\":"; json += modbusRelayTask.getWritesInjected();

        json += "}";
        request->send(200, "application/json", json);
    });

    // POST API: Control (write register)
    auto* ctrlHandler = new AsyncCallbackJsonWebHandler("/api/control");
    ctrlHandler->setMethod(HTTP_POST);
    ctrlHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        JsonObject root = json.as<JsonObject>();
        uint16_t reg = root["reg"] | 0;
        uint16_t value = root["value"] | 0;

        if (reg == 0)
        {
            request->send(400, "application/json", "{\"error\":\"reg required\"}");
            return;
        }

        optitronic2MqttTask.queueWrite(reg, value);
        request->send(200, "application/json", "{\"status\":\"queued\"}");
    });
    mServer.addHandler(ctrlHandler);

    // ===== EMS-ESP32 compatible REST endpoints for web UI =====

    // Dashboard data (tree table format)
    mServer.on("/rest/dashboardData", HTTP_GET, [](AsyncWebServerRequest* request) {
        using namespace aquamqtt::message::optitronic2;
        auto& state = aquamqtt::Optitronic2State::getInstance();

        // Build JSON matching EMS-ESP32 DashboardData format:
        // { connected: true, nodes: [ { id, t, n, nodes: [ { id, dv: { id, v, u, c, m, x, s, l } } ] } ] }
        // Device types: we use a generic type=5 for the heat pump
        // Entity IDs: first 2 hex chars are mask (00=readable, writable entities have mask+command)
        // UOM: 0=NONE, 1=DEGREES, 3=PERCENT, 7=HOURS, 8=MINUTES

        String json = "{\"connected\":true,\"nodes\":[";

        // --- Device 1: Heat Pump Status (id=1) ---
        json += "{\"id\":1,\"t\":5,\"n\":\"Heat Pump\",\"nodes\":[";
        int eid = 100;

        // Water Temperature
        json += "{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Water Temperature\",\"v\":";
        json += String(state.getWaterTemp(), 1);
        json += ",\"u\":1}}";

        // Ambient Temperature
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Ambient Temperature\",\"v\":";
        json += String(state.getAmbientTemp(), 1);
        json += ",\"u\":1}}";

        // Evaporator Temperature
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Evaporator Temperature\",\"v\":";
        json += String(state.getEvaporatorTemp(), 1);
        json += ",\"u\":1}}";

        // Operating State
        {
            const char* stateStr;
            switch (state.getOperatingState())
            {
                case 2: stateStr = "IDLE"; break;
                case 3: stateStr = "HEATING"; break;
                case 6: stateStr = "PV_BOOST"; break;
                default: stateStr = "UNKNOWN"; break;
            }
            json += ",{\"id\":"; json += eid++;
            json += ",\"dv\":{\"id\":\"00Operating State\",\"v\":\"";
            json += stateStr;
            json += "\"}}";
        }

        // Active Setpoint
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Active Setpoint\",\"v\":";
        json += String(state.getActiveSetpoint(), 1);
        json += ",\"u\":1}}";

        // Heat Source
        {
            const char* hsStr;
            switch (state.getHeatSource())
            {
                case 0: hsStr = "NONE"; break;
                case 1: hsStr = "HEATPUMP"; break;
                case 2: hsStr = "EXT_BOILER"; break;
                case 3: hsStr = "EXT_SOLAR"; break;
                default: hsStr = "UNKNOWN"; break;
            }
            json += ",{\"id\":"; json += eid++;
            json += ",\"dv\":{\"id\":\"00Heat Source\",\"v\":\"";
            json += hsStr;
            json += "\"}}";
        }

        // Quick Heat Active
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Quick Heat Active\",\"v\":\"";
        json += state.isQuickHeatActive() ? "ON" : "OFF";
        json += "\"}}";

        // Force Heating
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Force Heating\",\"v\":\"";
        json += state.isForceHeating() ? "ON" : "OFF";
        json += "\"}}";

        // PV Active
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00PV Active\",\"v\":\"";
        json += state.isPvActive() ? "ON" : "OFF";
        json += "\"}}";

        json += "]}";

        // --- Device 2: Settings (id=2) ---
        json += ",{\"id\":2,\"t\":5,\"n\":\"Settings\",\"nodes\":[";
        eid = 200;

        // DHW Setpoint (writable)
        json += "{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00DHW Setpoint\",\"v\":";
        json += String(state.getDhwSetpoint(), 1);
        json += ",\"u\":1,\"c\":\"dhwSetpoint\",\"m\":40,\"x\":65,\"s\":\"0.5\"}}";

        // Eco Deviation (writable)
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Eco Deviation\",\"v\":";
        json += String(state.getEcoDeviation(), 1);
        json += ",\"u\":1,\"c\":\"ecoDeviation\",\"m\":-10,\"x\":0,\"s\":\"0.5\"}}";

        // Komfort Deviation (writable)
        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Komfort Deviation\",\"v\":";
        json += String(state.getKomfortDeviation(), 1);
        json += ",\"u\":1,\"c\":\"komfortDeviation\",\"m\":0,\"x\":10,\"s\":\"0.5\"}}";

        // Program (writable select)
        {
            const char* progStr;
            switch (state.getProgram())
            {
                case 0: progStr = "ECO"; break;
                case 1: progStr = "KOMFORT"; break;
                case 2: progStr = "BOOST"; break;
                case 3: progStr = "AUTO"; break;
                default: progStr = "UNKNOWN"; break;
            }
            json += ",{\"id\":"; json += eid++;
            json += ",\"dv\":{\"id\":\"00Program\",\"v\":\"";
            json += progStr;
            json += "\",\"c\":\"program\",\"l\":[\"ECO\",\"KOMFORT\",\"BOOST\",\"AUTO\"]}}";
        }

        // Aux Heat Mode (writable select)
        {
            const char* auxStr;
            switch (state.getAuxHeatMode())
            {
                case 0: auxStr = "OFF"; break;
                case 1: auxStr = "ECO"; break;
                case 2: auxStr = "SMART_GRID"; break;
                default: auxStr = "UNKNOWN"; break;
            }
            json += ",{\"id\":"; json += eid++;
            json += ",\"dv\":{\"id\":\"00Aux Heat Mode\",\"v\":\"";
            json += auxStr;
            json += "\",\"c\":\"auxHeatMode\",\"l\":[\"OFF\",\"ECO\",\"SMART_GRID\"]}}";
        }

        // External Input Function (writable select)
        {
            const char* extStr;
            switch (state.getExtInputFunction())
            {
                case 0: extStr = "DISABLED"; break;
                case 1: extStr = "PV_FUNCTION"; break;
                case 2: extStr = "SG_READY"; break;
                default: extStr = "UNKNOWN"; break;
            }
            json += ",{\"id\":"; json += eid++;
            json += ",\"dv\":{\"id\":\"00External Input Function\",\"v\":\"";
            json += extStr;
            json += "\",\"c\":\"extInputFunction\",\"l\":[\"DISABLED\",\"PV_FUNCTION\",\"SG_READY\"]}}";
        }

        json += "]}";

        // --- Device 3: Installer Parameters (id=3) ---
        if (state.hasBlock(REG_INSTALLER_BLK1_START) || state.hasBlock(REG_INSTALLER_BLK2_START))
        {
            json += ",{\"id\":3,\"t\":5,\"n\":\"Installer\",\"nodes\":[";
            eid = 300;
            bool first = true;

            if (state.hasBlock(REG_INSTALLER_BLK1_START))
            {
                // Frost Protection Temp
                json += "{\"id\":"; json += eid++;
                json += ",\"dv\":{\"id\":\"00Frost Protection Temp\",\"v\":";
                json += String(state.getFrostProtectTemp(), 1);
                json += ",\"u\":1,\"c\":\"frostProtectTemp\",\"m\":-10,\"x\":10,\"s\":\"0.5\"}}";

                // Anti-Legionella Interval
                json += ",{\"id\":"; json += eid++;
                json += ",\"dv\":{\"id\":\"00Anti-Legionella Interval\",\"v\":";
                json += state.getAntiLegioInterval();
                json += ",\"u\":0,\"c\":\"antiLegioInterval\",\"m\":0,\"x\":90,\"s\":\"1\"}}";
                first = false;
            }

            if (state.hasBlock(REG_INSTALLER_BLK2_START))
            {
                if (!first) json += ",";
                // Bivalent Threshold
                json += "{\"id\":"; json += eid++;
                json += ",\"dv\":{\"id\":\"00Bivalent Threshold\",\"v\":";
                json += String(state.getBivalentThreshold(), 1);
                json += ",\"u\":1,\"c\":\"bivalentThreshold\",\"m\":-20,\"x\":20,\"s\":\"0.5\"}}";

                // PV Target Setpoint
                json += ",{\"id\":"; json += eid++;
                json += ",\"dv\":{\"id\":\"00PV Target Setpoint\",\"v\":";
                json += String(state.getPvTargetSetpoint(), 1);
                json += ",\"u\":1,\"c\":\"pvTargetSetpoint\",\"m\":40,\"x\":70,\"s\":\"0.5\"}}";

                // External Source Max Temp
                json += ",{\"id\":"; json += eid++;
                json += ",\"dv\":{\"id\":\"00External Source Max Temp\",\"v\":";
                json += String(state.getExtSourceMaxTemp(), 1);
                json += ",\"u\":1,\"c\":\"extSourceMaxTemp\",\"m\":20,\"x\":90,\"s\":\"0.5\"}}";

                // External Source Priority
                {
                    const char* prioStr;
                    switch (state.getExtSourcePriority())
                    {
                        case 0: prioStr = "DEVICE"; break;
                        case 1: prioStr = "EXTERNAL"; break;
                        default: prioStr = "UNKNOWN"; break;
                    }
                    json += ",{\"id\":"; json += eid++;
                    json += ",\"dv\":{\"id\":\"00External Source Priority\",\"v\":\"";
                    json += prioStr;
                    json += "\",\"c\":\"extSourcePriority\",\"l\":[\"DEVICE\",\"EXTERNAL\"]}}";
                }
            }

            json += "]}";
        }

        // --- Device 4: Diagnostics (id=4) ---
        json += ",{\"id\":4,\"t\":5,\"n\":\"Diagnostics\",\"nodes\":[";
        eid = 400;

        json += "{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Uptime\",\"v\":";
        json += (millis() / 1000);
        json += ",\"u\":14}}";  // UOM 14 = SECONDS

        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Frames Received\",\"v\":";
        json += modbusRelayTask.getFramesReceived();
        json += "}}";

        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00CRC Errors\",\"v\":";
        json += modbusRelayTask.getCrcErrors();
        json += "}}";

        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Writes Injected\",\"v\":";
        json += modbusRelayTask.getWritesInjected();
        json += "}}";

        json += ",{\"id\":"; json += eid++;
        json += ",\"dv\":{\"id\":\"00Free Heap\",\"v\":";
        json += (ESP.getFreeHeap() / 1024);
        json += ",\"u\":13}}";  // UOM 13 = KB

        json += "]}";

        json += "]}";
        request->send(200, "application/json", json);
    });

    // Core data endpoint (device list)
    mServer.on("/rest/coreData", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"connected\":true,\"devices\":[{\"id\":1,\"tn\":\"Heat Pump\",\"t\":5,\"b\":\"Atlantic\",\"n\":\"Optitronic 2\",\"d\":1,\"p\":1,\"v\":\"2.0\",\"e\":20}]}");
    });

    // Device data endpoint (entity list for a device)
    mServer.on("/rest/deviceData", HTTP_GET, [](AsyncWebServerRequest* request) {
        // Return empty nodes array - devices page will show device info from coreData
        request->send(200, "application/json", "{\"nodes\":[]}");
    });

    // Write device value (from dashboard edit dialog)
    auto* writeHandler = new AsyncCallbackJsonWebHandler("/rest/writeDeviceValue");
    writeHandler->setMethod(HTTP_POST);
    writeHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        using namespace aquamqtt::message::optitronic2;
        JsonObject root = json.as<JsonObject>();
        const char* cmd = root["c"] | "";
        JsonVariant val = root["v"];

        if (strlen(cmd) == 0)
        {
            request->send(400, "application/json", "{\"error\":\"command required\"}");
            return;
        }

        // Map command names to register addresses and value encoding
        uint16_t reg = 0;
        uint16_t regVal = 0;

        if (strcmp(cmd, "dhwSetpoint") == 0)
        {
            reg = REG_DHW_SETPOINT;
            regVal = (uint16_t)(val.as<float>() * 10.0f);
        }
        else if (strcmp(cmd, "ecoDeviation") == 0)
        {
            reg = REG_ECO_DEVIATION;
            regVal = (uint16_t)((int16_t)(val.as<float>() * 10.0f));
        }
        else if (strcmp(cmd, "komfortDeviation") == 0)
        {
            reg = REG_KOMFORT_DEVIATION;
            regVal = (uint16_t)((int16_t)(val.as<float>() * 10.0f));
        }
        else if (strcmp(cmd, "program") == 0)
        {
            reg = REG_PROGRAM;
            String sv = val.as<String>();
            if (sv == "ECO") regVal = 0;
            else if (sv == "KOMFORT") regVal = 1;
            else if (sv == "BOOST") regVal = 2;
            else if (sv == "AUTO") regVal = 3;
        }
        else if (strcmp(cmd, "auxHeatMode") == 0)
        {
            reg = REG_AUX_HEAT_MODE;
            String sv = val.as<String>();
            if (sv == "OFF") regVal = 0;
            else if (sv == "ECO") regVal = 1;
            else if (sv == "SMART_GRID") regVal = 2;
        }
        else if (strcmp(cmd, "extInputFunction") == 0)
        {
            reg = REG_EXT_INPUT_FUNCTION;
            String sv = val.as<String>();
            if (sv == "DISABLED") regVal = 0;
            else if (sv == "PV_FUNCTION") regVal = 1;
            else if (sv == "SG_READY") regVal = 2;
        }
        else if (strcmp(cmd, "frostProtectTemp") == 0)
        {
            reg = REG_FROST_PROTECT_TEMP;
            regVal = (uint16_t)((int16_t)(val.as<float>() * 10.0f));
        }
        else if (strcmp(cmd, "antiLegioInterval") == 0)
        {
            reg = REG_ANTI_LEGIO_INTERVAL;
            regVal = val.as<uint16_t>();
        }
        else if (strcmp(cmd, "bivalentThreshold") == 0)
        {
            reg = REG_BIVALENT_THRESHOLD;
            regVal = (uint16_t)((int16_t)(val.as<float>() * 10.0f));
        }
        else if (strcmp(cmd, "pvTargetSetpoint") == 0)
        {
            reg = REG_PV_TARGET_SETPOINT;
            regVal = (uint16_t)(val.as<float>() * 10.0f);
        }
        else if (strcmp(cmd, "extSourceMaxTemp") == 0)
        {
            reg = REG_EXT_SOURCE_MAX_TEMP;
            regVal = (uint16_t)(val.as<float>() * 10.0f);
        }
        else if (strcmp(cmd, "extSourcePriority") == 0)
        {
            reg = REG_EXT_SOURCE_PRIORITY;
            String sv = val.as<String>();
            if (sv == "DEVICE") regVal = 0;
            else if (sv == "EXTERNAL") regVal = 1;
        }
        else
        {
            request->send(400, "application/json", "{\"error\":\"unknown command\"}");
            return;
        }

        if (reg != 0)
        {
            optitronic2MqttTask.queueWrite(reg, regVal);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
        else
        {
            request->send(400, "application/json", "{\"error\":\"invalid register\"}");
        }
    });
    mServer.addHandler(writeHandler);

    // ===== Stub REST endpoints for all EMS-ESP32 web UI pages =====

    // Sensors page
    mServer.on("/rest/sensorData", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"ts\":[],\"as\":[],\"analog_enabled\":false,\"available_gpios\":[],\"exclude_types\":[],\"platform\":\"ESP32-S3\"}");
    });

    // Activity page
    mServer.on("/rest/activity", HTTP_GET, [](AsyncWebServerRequest* request) {
        auto& state = aquamqtt::Optitronic2State::getInstance();
        String json = "{\"stats\":[{\"id\":0,\"s\":";
        json += modbusRelayTask.getFramesReceived();
        json += ",\"f\":";
        json += modbusRelayTask.getCrcErrors();
        uint32_t total = modbusRelayTask.getFramesReceived() + modbusRelayTask.getCrcErrors();
        uint16_t quality = total > 0 ? (uint16_t)(modbusRelayTask.getFramesReceived() * 100 / total) : 0;
        json += ",\"q\":";
        json += quality;
        json += "}]}";
        request->send(200, "application/json", json);
    });

    // Application Settings
    mServer.on("/rest/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"locale\":\"en\",\"tx_mode\":0,\"ems_bus_id\":0,"
            "\"syslog_enabled\":false,\"syslog_level\":3,\"syslog_mark_interval\":0,"
            "\"syslog_host\":\"\",\"syslog_port\":514,"
            "\"boiler_heatingoff\":false,\"remote_timeout_en\":false,\"remote_timeout\":0,"
            "\"shower_timer\":false,\"shower_alert\":false,"
            "\"shower_alert_coldshot\":10,\"shower_alert_trigger\":7,\"shower_min_duration\":120,"
            "\"rx_gpio\":0,\"tx_gpio\":0,\"telnet_enabled\":false,"
            "\"dallas_gpio\":0,\"dallas_parasite\":false,"
            "\"led_gpio\":0,\"led_type\":0,\"hide_led\":false,"
            "\"low_clock\":false,\"notoken_api\":true,\"readonly_mode\":false,"
            "\"analog_enabled\":false,\"pbutton_gpio\":0,\"trace_raw\":false,"
            "\"board_profile\":\"S3\",\"bool_format\":1,\"bool_dashboard\":1,"
            "\"enum_format\":1,\"fahrenheit\":false,"
            "\"phy_type\":0,\"eth_power\":0,\"eth_phy_addr\":0,\"eth_clock_mode\":0,"
            "\"platform\":\"ESP32-S3\","
            "\"modbus_enabled\":true,\"modbus_port\":502,\"modbus_max_clients\":1,\"modbus_timeout\":2000,"
            "\"developer_mode\":false}");
    });

    // Scheduler
    mServer.on("/rest/schedule", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"schedule\":[]}");
    });

    // Modules
    mServer.on("/rest/modules", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"modules\":[]}");
    });

    // Device entities (Customizations)
    mServer.on("/rest/deviceEntities", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "[]");
    });

    // Custom entities
    mServer.on("/rest/customEntities", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"entities\":[]}");
    });

    // MQTT Status
    mServer.on("/rest/mqttStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
        // Read MQTT config to report
        String json = "{\"enabled\":true,\"connected\":true,\"client_id\":\"aquamqtt\","
            "\"disconnect_reason\":0,\"mqtt_fails\":0,\"mqtt_queued\":0,\"connect_count\":1}";
        request->send(200, "application/json", json);
    });

    // MQTT Settings
    mServer.on("/rest/mqttSettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        // Read from config file
        if (LittleFS.exists("/config/mqtt.json"))
        {
            File f = LittleFS.open("/config/mqtt.json", "r");
            String cfg = f.readString();
            f.close();

            // Parse and map to EMS-ESP format
            JsonDocument doc;
            deserializeJson(doc, cfg);

            String json = "{\"enabled\":true,\"host\":\"";
            json += doc["server"].as<String>();
            json += "\",\"port\":";
            json += doc["port"] | 1883;
            json += ",\"base\":\"aquamqtt\",\"username\":\"";
            json += doc["user"].as<String>();
            json += "\",\"password\":\"";
            json += doc["password"].as<String>();
            json += "\",\"client_id\":\"";
            json += doc["clientId"].as<String>();
            json += "\",\"keep_alive\":60,\"clean_session\":true,\"entity_format\":1,"
                "\"publish_time_boiler\":10,\"publish_time_thermostat\":10,"
                "\"publish_time_solar\":10,\"publish_time_mixer\":10,"
                "\"publish_time_water\":10,\"publish_time_other\":10,"
                "\"publish_time_sensor\":10,\"publish_time_heartbeat\":60,"
                "\"mqtt_qos\":0,\"mqtt_retain\":false,"
                "\"ha_enabled\":";
            json += (doc["enableDiscovery"] | false) ? "true" : "false";
            json += ",\"nested_format\":1,\"send_response\":false,"
                "\"publish_single\":false,\"publish_single2cmd\":false,"
                "\"discovery_prefix\":\"";
            json += doc["discoveryPrefix"].as<String>();
            json += "\",\"discovery_type\":0,\"ha_number_mode\":0}";
            request->send(200, "application/json", json);
        }
        else
        {
            request->send(200, "application/json",
                "{\"enabled\":true,\"host\":\"\",\"port\":1883,\"base\":\"aquamqtt\","
                "\"username\":\"\",\"password\":\"\",\"client_id\":\"aquamqtt\","
                "\"keep_alive\":60,\"clean_session\":true,\"entity_format\":1,"
                "\"publish_time_boiler\":10,\"publish_time_thermostat\":10,"
                "\"publish_time_solar\":10,\"publish_time_mixer\":10,"
                "\"publish_time_water\":10,\"publish_time_other\":10,"
                "\"publish_time_sensor\":10,\"publish_time_heartbeat\":60,"
                "\"mqtt_qos\":0,\"mqtt_retain\":false,"
                "\"ha_enabled\":false,\"nested_format\":1,\"send_response\":false,"
                "\"publish_single\":false,\"publish_single2cmd\":false,"
                "\"discovery_prefix\":\"homeassistant\",\"discovery_type\":0,\"ha_number_mode\":0}");
        }
    });

    // Network Status
    mServer.on("/rest/networkStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"status\":3,\"local_ip\":\"";
        json += WiFi.localIP().toString();
        json += "\",\"local_ipv6\":\"::\",\"mac_address\":\"";
        json += WiFi.macAddress();
        json += "\",\"rssi\":";
        json += WiFi.RSSI();
        json += ",\"ssid\":\"";
        json += WiFi.SSID();
        json += "\",\"bssid\":\"";
        json += WiFi.BSSIDstr();
        json += "\",\"channel\":";
        json += WiFi.channel();
        json += ",\"subnet_mask\":\"";
        json += WiFi.subnetMask().toString();
        json += "\",\"gateway_ip\":\"";
        json += WiFi.gatewayIP().toString();
        json += "\",\"dns_ip_1\":\"";
        json += WiFi.dnsIP(0).toString();
        json += "\",\"dns_ip_2\":\"";
        json += WiFi.dnsIP(1).toString();
        json += "\",\"hostname\":\"aquamqtt\",\"reconnect_count\":0}";
        request->send(200, "application/json", json);
    });

    // Network Settings
    mServer.on("/rest/networkSettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (LittleFS.exists("/config/wifi.json"))
        {
            File f = LittleFS.open("/config/wifi.json", "r");
            String cfg = f.readString();
            f.close();

            JsonDocument doc;
            deserializeJson(doc, cfg);

            String json = "{\"ssid\":\"";
            json += doc["ssid"].as<String>();
            json += "\",\"bssid\":\"\",\"password\":\"";
            json += doc["password"].as<String>();
            json += "\",\"hostname\":\"";
            json += doc["networkName"] | "aquamqtt";
            json += "\",\"static_ip_config\":false,"
                "\"bandwidth20\":false,\"nosleep\":false,\"tx_power\":20,"
                "\"enableMDNS\":true,\"enableCORS\":false,\"CORSOrigin\":\"\"}";
            request->send(200, "application/json", json);
        }
        else
        {
            request->send(200, "application/json",
                "{\"ssid\":\"\",\"bssid\":\"\",\"password\":\"\",\"hostname\":\"aquamqtt\","
                "\"static_ip_config\":false,\"bandwidth20\":false,\"nosleep\":false,"
                "\"tx_power\":20,\"enableMDNS\":true,\"enableCORS\":false,\"CORSOrigin\":\"\"}");
        }
    });

    // NTP Status
    mServer.on("/rest/ntpStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"status\":0,\"utc_time\":\"\",\"local_time\":\"Not configured\",\"server\":\"pool.ntp.org\"}");
    });

    // NTP Settings
    mServer.on("/rest/ntpSettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"enabled\":false,\"server\":\"pool.ntp.org\","
            "\"tz_label\":\"Europe/Vienna\",\"tz_format\":\"CET-1CEST,M3.5.0,M10.5.0/3\"}");
    });

    // AP Status
    mServer.on("/rest/apStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"status\":0,\"ip_address\":\"192.168.4.1\",\"mac_address\":\"00:00:00:00:00:00\",\"station_num\":0}");
    });

    // AP Settings
    mServer.on("/rest/apSettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"provision_mode\":0,\"ssid\":\"AquaMQTT\",\"password\":\"\","
            "\"channel\":1,\"ssid_hidden\":false,\"max_clients\":4,"
            "\"local_ip\":\"192.168.4.1\",\"gateway_ip\":\"192.168.4.1\",\"subnet_mask\":\"255.255.255.0\"}");
    });

    // Security Settings
    mServer.on("/rest/securitySettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"users\":[{\"username\":\"admin\",\"password\":\"admin\",\"admin\":true}],\"jwt_secret\":\"aquamqtt-secret\"}");
    });

    // System Status (used by Hardware Status, Version, DownloadUpload)
    mServer.on("/rest/systemStatus", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"emsesp_version\":\"2.0.0-aquamqtt\",\"bus_status\":0,\"uptime\":";
        json += (millis() / 1000);
        json += ",\"bus_uptime\":";
        json += (millis() / 1000);
        json += ",\"num_devices\":1,\"num_sensors\":3,\"num_analogs\":0,"
            "\"ntp_status\":0,\"ntp_time\":\"\","
            "\"mqtt_status\":true,\"ap_status\":false,"
            "\"network_status\":3,\"wifi_rssi\":";
        json += WiFi.RSSI();
        json += ",\"build_flags\":\"\",\"esp_platform\":\"ESP32-S3\","
            "\"max_alloc_heap\":";
        json += ESP.getMaxAllocHeap();
        json += ",\"cpu_type\":\"ESP32-S3\",\"cpu_rev\":0,\"cpu_cores\":2,\"cpu_freq_mhz\":240,"
            "\"free_heap\":";
        json += ESP.getFreeHeap();
        json += ",\"arduino_version\":\"3.x\",\"sdk_version\":\"5.x\","
            "\"partition\":\"app0\",\"flash_chip_size\":";
        json += ESP.getFlashChipSize();
        json += ",\"flash_chip_speed\":";
        json += ESP.getFlashChipSpeed();
        json += ",\"app_used\":";
        json += ESP.getSketchSize();
        json += ",\"app_free\":";
        json += ESP.getFreeSketchSpace();
        json += ",\"fs_used\":";
        json += LittleFS.usedBytes();
        json += ",\"fs_free\":";
        json += (LittleFS.totalBytes() - LittleFS.usedBytes());
        json += ",\"free_mem\":";
        json += ESP.getFreeHeap();
        json += ",\"psram\":false,\"free_caps\":";
        json += ESP.getMaxAllocHeap();
        json += ",\"model\":\"AquaMQTT Optitronic 2\",\"board\":\"ESP32-S3-Nano\","
            "\"has_loader\":false,\"has_partition\":false,"
            "\"partitions\":[],\"status\":0,\"developer_mode\":false}";
        request->send(200, "application/json", json);
    });

    // Log Settings
    mServer.on("/rest/logSettings", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"level\":6,\"max_messages\":50,\"compact\":false,\"psram\":false,\"developer_mode\":false}");
    });

    // Features (used by various pages)
    mServer.on("/rest/features", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"security\":true,\"mqtt\":true,\"ntp\":false,\"ota\":false,\"upload_firmware\":false}");
    });

    // Verify Authorization (always OK since no auth)
    mServer.on("/rest/verifyAuthorization", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"access_token\":\"aquamqtt-admin\"}");
    });

    // Sign In (always succeed)
    mServer.on("/rest/signIn", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"access_token\":\"aquamqtt-admin\"}");
    });

    // Board Profile
    mServer.on("/rest/boardProfile", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json",
            "{\"board\":\"S3\",\"led_gpio\":0,\"dallas_gpio\":0,\"rx_gpio\":0,\"tx_gpio\":0,"
            "\"pbutton_gpio\":0,\"phy_type\":0,\"eth_power\":0,\"eth_phy_addr\":0,\"eth_clock_mode\":0}");
    });

    // WiFi scan
    mServer.on("/rest/scanNetworks", HTTP_GET, [](AsyncWebServerRequest* request) {
        WiFi.scanNetworks(true);
        request->send(200, "application/json", "{\"status\":\"scanning\"}");
    });

    // WiFi scan results
    mServer.on("/rest/listNetworks", HTTP_GET, [](AsyncWebServerRequest* request) {
        int n = WiFi.scanComplete();
        if (n < 0)
        {
            request->send(200, "application/json", "{\"networks\":[]}");
            return;
        }
        String json = "{\"networks\":[";
        for (int i = 0; i < n; i++)
        {
            if (i > 0) json += ",";
            json += "{\"rssi\":";
            json += WiFi.RSSI(i);
            json += ",\"ssid\":\"";
            json += WiFi.SSID(i);
            json += "\",\"bssid\":\"";
            json += WiFi.BSSIDstr(i);
            json += "\",\"channel\":";
            json += WiFi.channel(i);
            json += ",\"encryption_type\":";
            json += WiFi.encryptionType(i);
            json += "}";
        }
        json += "]}";
        WiFi.scanDelete();
        request->send(200, "application/json", json);
    });

    // Generic POST stubs for pages that write settings
    auto* settingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/settings");
    settingsPostHandler->setMethod(HTTP_POST);
    settingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(settingsPostHandler);

    auto* schedulePostHandler = new AsyncCallbackJsonWebHandler("/rest/schedule");
    schedulePostHandler->setMethod(HTTP_POST);
    schedulePostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(schedulePostHandler);

    auto* modulesPostHandler = new AsyncCallbackJsonWebHandler("/rest/modules");
    modulesPostHandler->setMethod(HTTP_POST);
    modulesPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(modulesPostHandler);

    auto* mqttSettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/mqttSettings");
    mqttSettingsPostHandler->setMethod(HTTP_POST);
    mqttSettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        // Save relevant fields to our mqtt config
        JsonObject root = json.as<JsonObject>();
        JsonDocument doc;
        doc["server"] = root["host"] | "";
        doc["port"] = root["port"] | 1883;
        doc["user"] = root["username"] | "";
        doc["password"] = root["password"] | "";
        doc["clientId"] = root["client_id"] | "aquamqtt";
        doc["enableDiscovery"] = root["ha_enabled"] | false;
        doc["discoveryPrefix"] = root["discovery_prefix"] | "homeassistant";

        String output;
        serializeJson(doc, output);
        WebHandler::saveConfigFile("mqtt", output);
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(mqttSettingsPostHandler);

    auto* networkSettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/networkSettings");
    networkSettingsPostHandler->setMethod(HTTP_POST);
    networkSettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        JsonObject root = json.as<JsonObject>();
        JsonDocument doc;
        doc["ssid"] = root["ssid"] | "";
        doc["password"] = root["password"] | "";
        doc["networkName"] = root["hostname"] | "aquamqtt";

        String output;
        serializeJson(doc, output);
        WebHandler::saveConfigFile("wifi", output);
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(networkSettingsPostHandler);

    auto* ntpSettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/ntpSettings");
    ntpSettingsPostHandler->setMethod(HTTP_POST);
    ntpSettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(ntpSettingsPostHandler);

    auto* apSettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/apSettings");
    apSettingsPostHandler->setMethod(HTTP_POST);
    apSettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(apSettingsPostHandler);

    auto* securitySettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/securitySettings");
    securitySettingsPostHandler->setMethod(HTTP_POST);
    securitySettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(securitySettingsPostHandler);

    auto* logSettingsPostHandler = new AsyncCallbackJsonWebHandler("/rest/logSettings");
    logSettingsPostHandler->setMethod(HTTP_POST);
    logSettingsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(logSettingsPostHandler);

    auto* actionPostHandler = new AsyncCallbackJsonWebHandler("/rest/action");
    actionPostHandler->setMethod(HTTP_POST);
    actionPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(actionPostHandler);

    auto* apiPostHandler = new AsyncCallbackJsonWebHandler("/api");
    apiPostHandler->setMethod(HTTP_POST);
    apiPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(apiPostHandler);

    auto* signInPostHandler = new AsyncCallbackJsonWebHandler("/rest/signIn");
    signInPostHandler->setMethod(HTTP_POST);
    signInPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"access_token\":\"aquamqtt-admin\"}");
    });
    mServer.addHandler(signInPostHandler);

    auto* customEntitiesPostHandler = new AsyncCallbackJsonWebHandler("/rest/customEntities");
    customEntitiesPostHandler->setMethod(HTTP_POST);
    customEntitiesPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(customEntitiesPostHandler);

    auto* customizationEntitiesPostHandler = new AsyncCallbackJsonWebHandler("/rest/customizationEntities");
    customizationEntitiesPostHandler->setMethod(HTTP_POST);
    customizationEntitiesPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(customizationEntitiesPostHandler);

    auto* resetCustomizationsPostHandler = new AsyncCallbackJsonWebHandler("/rest/resetCustomizations");
    resetCustomizationsPostHandler->setMethod(HTTP_POST);
    resetCustomizationsPostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(resetCustomizationsPostHandler);

    auto* writeDeviceNamePostHandler = new AsyncCallbackJsonWebHandler("/rest/writeDeviceName");
    writeDeviceNamePostHandler->setMethod(HTTP_POST);
    writeDeviceNamePostHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    mServer.addHandler(writeDeviceNamePostHandler);
    auto* wifiHandler = new AsyncCallbackJsonWebHandler("/api/wifi");
    wifiHandler->setMethod(HTTP_POST);
    wifiHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        JsonObject root = json.as<JsonObject>();
        bool reboot = root["reboot"] | false;
        JsonObject data = root["data"].as<JsonObject>();

        String ssid = data["ssid"] | "";
        if (ssid.length() == 0)
        {
            request->send(400, "application/json", "{\"error\":\"SSID required\"}");
            return;
        }

        String output;
        serializeJson(data, output);
        if (saveConfigFile("wifi", output))
        {
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
        else
        {
            request->send(500, "application/json", "{\"error\":\"write failed\"}");
            return;
        }

        if (reboot)
        {
            delay(1000);
            ESP.restart();
        }
    });

    // POST API: MQTT config
    auto* mqttHandler = new AsyncCallbackJsonWebHandler("/api/mqtt");
    mqttHandler->setMethod(HTTP_POST);
    mqttHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        JsonObject root = json.as<JsonObject>();
        bool reboot = root["reboot"] | false;
        JsonObject data = root["data"].as<JsonObject>();

        String output;
        serializeJson(data, output);
        if (saveConfigFile("mqtt", output))
        {
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
        else
        {
            request->send(500, "application/json", "{\"error\":\"write failed\"}");
            return;
        }

        if (reboot)
        {
            delay(1000);
            ESP.restart();
        }
    });

    // POST API: AquaMQTT config
    auto* aquaHandler = new AsyncCallbackJsonWebHandler("/api/aquamqtt");
    aquaHandler->setMethod(HTTP_POST);
    aquaHandler->onRequest([](AsyncWebServerRequest* request, JsonVariant& json) {
        JsonObject root = json.as<JsonObject>();
        bool reboot = root["reboot"] | false;
        JsonObject data = root["data"].as<JsonObject>();

        String output;
        serializeJson(data, output);
        if (saveConfigFile("aquamqtt", output))
        {
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
        else
        {
            request->send(500, "application/json", "{\"error\":\"write failed\"}");
            return;
        }

        if (reboot)
        {
            delay(1000);
            ESP.restart();
        }
    });

    mServer.addHandler(wifiHandler);
    mServer.addHandler(mqttHandler);
    mServer.addHandler(aquaHandler);

    mServer.onNotFound(handleNotFound);
    mServer.begin();
    Serial.println("[web] server started on port 80");
}

void WebHandler::loop()
{
}

bool WebHandler::saveConfigFile(const String& filename, const String& content)
{
    String path = "/config/" + filename + ".json";
    File file = LittleFS.open(path, "w");
    if (!file)
    {
        Serial.printf("[web] failed to open %s for writing\n", path.c_str());
        return false;
    }
    file.print(content);
    file.close();
    return true;
}

void WebHandler::handleRoot(AsyncWebServerRequest* request)
{
    if (LittleFS.exists("/index.html"))
    {
        request->send(LittleFS, "/index.html", "text/html");
    }
    else
    {
        // Serve a minimal embedded page if no filesystem HTML available
        request->send(200, "text/html",
            "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>AquaMQTT Config</title><style>body{font-family:sans-serif;background:#1a1a2e;color:#e0e0e0;padding:20px;max-width:600px;margin:0 auto}"
            "h1{color:#4fc3f7}label{display:block;margin:12px 0 4px;color:#90caf9;font-size:.85em}"
            "input,select{width:100%;padding:8px;border:1px solid #333;border-radius:4px;background:#0f3460;color:#e0e0e0;box-sizing:border-box}"
            ".tabs{display:flex;gap:4px;margin:20px 0}.tab{padding:8px 16px;background:#16213e;border:1px solid #333;border-radius:4px;cursor:pointer;color:#aaa}"
            ".tab.active{background:#0f3460;color:#4fc3f7}.panel{display:none}.panel.active{display:block}"
            ".btn{padding:10px 20px;border:none;border-radius:4px;cursor:pointer;font-weight:600;margin:4px}"
            ".btn-save{background:#4fc3f7;color:#000}.btn-reboot{background:#ff7043;color:#fff}"
            "#alert{padding:10px;border-radius:4px;margin:10px 0;display:none}</style></head><body>"
            "<h1>AquaMQTT Config</h1><p style='color:#888'>Optitronic 2</p><div id='alert'></div>"
            "<div class='tabs'><div class='tab active' data-tab='wifi'>WiFi</div><div class='tab' data-tab='mqtt'>MQTT</div><div class='tab' data-tab='aquamqtt'>Device</div></div>"
            "<div class='panel active' id='panel-wifi'><form id='form-wifi'>"
            "<label>SSID</label><input id='ssid' name='ssid'>"
            "<label>Password</label><input id='password' name='password' type='password'>"
            "<label>Network Name</label><input id='networkName' name='networkName'>"
            "</form></div>"
            "<div class='panel' id='panel-mqtt'><form id='form-mqtt'>"
            "<label>Server</label><input id='server' name='server'>"
            "<label>Port</label><input id='port' name='port' type='number'>"
            "<label>User</label><input id='user' name='user'>"
            "<label>Password</label><input id='mqtt_password' name='password' type='password'>"
            "<label>Client ID</label><input id='clientId' name='clientId'>"
            "<label><input id='enableDiscovery' name='enableDiscovery' type='checkbox'> HA Discovery</label>"
            "<label>Discovery Prefix</label><input id='discoveryPrefix' name='discoveryPrefix'>"
            "</form></div>"
            "<div class='panel' id='panel-aquamqtt'><form id='form-aquamqtt'>"
            "<label>Model Name</label><input id='heatpumpModelName' name='heatpumpModelName'>"
            "<label>Mode</label><select id='operationMode' name='operationMode'>"
            "<option value='0'>LISTENER</option><option value='1'>MITM</option>"
            "<option value='2'>OPTITRONIC2_LISTENER</option><option value='3'>OPTITRONIC2_MITM</option></select>"
            "</form></div>"
            "<div><button class='btn btn-save' id='btn-save'>Save</button>"
            "<button class='btn btn-reboot' id='btn-save-reboot'>Save &amp; Reboot</button></div>"
            "<script>"
            "document.querySelectorAll('.tab').forEach(t=>t.onclick=()=>{"
            "document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));"
            "document.querySelectorAll('.panel').forEach(x=>x.classList.remove('active'));"
            "t.classList.add('active');document.getElementById('panel-'+t.dataset.tab).classList.add('active')});"
            "async function load(){try{let w=await(await fetch('/api/wifi')).json();"
            "if(w.ssid)document.getElementById('ssid').value=w.ssid;"
            "if(w.password)document.getElementById('password').value=w.password;"
            "if(w.networkName)document.getElementById('networkName').value=w.networkName}catch(e){}"
            "try{let m=await(await fetch('/api/mqtt')).json();"
            "if(m.server)document.getElementById('server').value=m.server;"
            "if(m.port)document.getElementById('port').value=m.port;"
            "if(m.user)document.getElementById('user').value=m.user;"
            "if(m.password)document.getElementById('mqtt_password').value=m.password;"
            "if(m.clientId)document.getElementById('clientId').value=m.clientId;"
            "document.getElementById('enableDiscovery').checked=m.enableDiscovery||false;"
            "if(m.discoveryPrefix)document.getElementById('discoveryPrefix').value=m.discoveryPrefix}catch(e){}"
            "try{let a=await(await fetch('/api/aquamqtt')).json();"
            "if(a.heatpumpModelName)document.getElementById('heatpumpModelName').value=a.heatpumpModelName;"
            "if(a.operationMode!==undefined)document.getElementById('operationMode').value=a.operationMode}catch(e){}}"
            "function getEP(){let a=document.querySelector('.tab.active');return a?a.dataset.tab:'wifi'}"
            "function collect(ep){let f=document.getElementById('form-'+ep),d={};f.querySelectorAll('input,select').forEach(e=>{"
            "let k=e.name||e.id;if(e.type==='checkbox')d[k]=e.checked;else if(e.type==='number'||e.tagName==='SELECT')d[k]=parseInt(e.value)||0;else d[k]=e.value});return d}"
            "async function save(r){let ep=getEP(),d=collect(ep);try{let res=await fetch('/api/'+ep,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reboot:r,data:d})});"
            "let al=document.getElementById('alert');if(res.ok){al.style.display='block';al.style.background='#1b5e20';al.textContent='Saved'+(r?' - rebooting...':'')}"
            "else{al.style.display='block';al.style.background='#b71c1c';al.textContent='Error'}}catch(e){"
            "let al=document.getElementById('alert');al.style.display='block';al.style.background=r?'#1b5e20':'#b71c1c';al.textContent=r?'Rebooting...':'Connection error'}}"
            "document.getElementById('btn-save').onclick=()=>save(false);"
            "document.getElementById('btn-save-reboot').onclick=()=>save(true);load();"
            "</script></body></html>");
    }
}

void WebHandler::handleNotFound(AsyncWebServerRequest* request)
{
    // SPA fallback: serve index.html for non-API routes (client-side routing)
    String uri = request->url();
    if (!uri.startsWith("/api/") && !uri.startsWith("/rest/") && !uri.startsWith("/assets/")
        && !uri.startsWith("/css/") && !uri.startsWith("/fonts/") && !uri.startsWith("/app/"))
    {
        if (LittleFS.exists("/index.html"))
        {
            request->send(LittleFS, "/index.html", "text/html");
            return;
        }
    }
    request->send(404, "text/plain", "Not Found");
}

void WebHandler::handleWifiGet(AsyncWebServerRequest* request)
{
    request->send(LittleFS, "/config/wifi.json", "application/json");
}

void WebHandler::handleMqttGet(AsyncWebServerRequest* request)
{
    request->send(LittleFS, "/config/mqtt.json", "application/json");
}

void WebHandler::handleAquaGet(AsyncWebServerRequest* request)
{
    request->send(LittleFS, "/config/aquamqtt.json", "application/json");
}

void WebHandler::handleReboot(AsyncWebServerRequest* request)
{
    request->send(200, "text/plain", "Rebooting...");
    delay(1000);
    ESP.restart();
}

}  // namespace aquamqtt
