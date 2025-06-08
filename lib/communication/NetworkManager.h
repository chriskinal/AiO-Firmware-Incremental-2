#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "Arduino.h"
#include "mongoose.h"
#include "../system/DiagnosticManager.h"
#include "../config/ConfigManager.h"

class NetworkManager
{
public:
    NetworkManager(ConfigManager &configManager, DiagnosticManager &diagnosticManager);
    ~NetworkManager();

    bool begin();
    void update();
    bool isNetworkReady() const;

    // HTTP/GUI support
    void handleHTTPRequest(struct mg_connection *c, int ev, void *ev_data);
    void serveStaticFiles(struct mg_connection *c, struct mg_http_message *hm);
    void handleAPIRequests(struct mg_connection *c, struct mg_http_message *hm);

    // PGN and RTCM message handling
    bool sendPGNMessage(uint32_t pgn, const uint8_t *data, uint8_t length);
    bool sendPGN(uint32_t pgn, const uint8_t *data, uint8_t length) { return sendPGNMessage(pgn, data, length); }
    bool sendRTCMMessage(const uint8_t *data, uint16_t length);
    void getNetworkStats(uint32_t &sent, uint32_t &received);

    // Packet statistics methods
    uint32_t getPacketsSent() const { return packetsSent_; }
    uint32_t getPacketsReceived() const { return packetsReceived_; }

    // Handler setup methods
    void setPGNHandler(void (*handler)(struct mg_connection *c, int ev, void *ev_data))
    {
        pgnHandler_ = handler;
    }

    void setRTCMHandler(void (*handler)(struct mg_connection *c, int ev, void *ev_data))
    {
        rtcmHandler_ = handler;
    }

private:
    // Configuration and diagnostics
    ConfigManager &configManager_;
    DiagnosticManager &diagnosticManager_;

    // Network state
    bool networkReady_;

    // Mongoose connections
    struct mg_connection *pgnListener_;
    struct mg_connection *rtcmListener_;
    struct mg_connection *agioSender_;
    struct mg_connection *httpListener_; // For GUI

    // Statistics (public access needed by event handlers)
public:
    uint32_t packetsSent_;
    uint32_t packetsReceived_;
    uint32_t lastAgioConnection_;

    // Event handlers (public access needed by event handlers)
    void (*pgnHandler_)(struct mg_connection *c, int ev, void *ev_data);
    void (*rtcmHandler_)(struct mg_connection *c, int ev, void *ev_data);

private:
};

#endif // NETWORK_MANAGER_H