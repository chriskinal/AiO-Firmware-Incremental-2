#include "NetworkManager.h"

// Global Mongoose manager (required by Mongoose)
struct mg_mgr mgr;

// Provide the Mongoose glue functions directly
uint64_t mg_millis(void)
{
    return millis();
}

bool mg_random(void *buf, size_t len)
{
    uint8_t *dst = (uint8_t *)buf;
    // Use Teensy's true random number generator
    for (size_t i = 0; i < len; i++)
    {
        uint32_t val = TRNG_MCTL;
        TRNG_MCTL = val | TRNG_MCTL_TRNG_ACC | TRNG_MCTL_RST_DEF;
        while ((TRNG_MCTL & TRNG_MCTL_TRNG_ACC) != 0)
        {
            // Wait for accumulation to complete
        }
        dst[i] = TRNG_ENT0;
    }
    return true;
}

// Simplified ethernet initialization function
void ethernet_init(void)
{
    Serial.print("\r\n  - Starting Teensy 4.1 ethernet initialization");

    // Step 1: Configure PLL6 for 50 MHz - RMII reference clock
    CCM_ANALOG_PLL_ENET = CCM_ANALOG_PLL_ENET_BYPASS | CCM_ANALOG_PLL_ENET_ENABLE |
                          CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN |
                          CCM_ANALOG_PLL_ENET_ENET2_REF_EN;

    // Step 2: Configure pins for RMII
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_04 = 3; // ENET_RX_DATA00
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_05 = 3; // ENET_RX_DATA01
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_06 = 3; // ENET_RX_EN
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_07 = 3; // ENET_TX_DATA00
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_08 = 3; // ENET_TX_DATA01
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_09 = 3; // ENET_TX_EN
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_10 = 6; // ENET_REF_CLK
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_11 = 3; // ENET_RX_ER

    // Step 3: Set pin electrical properties
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = 0xB0E9; // RX_DATA00
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = 0xB0E9; // RX_DATA01
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = 0xB0E9; // RX_EN
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_07 = 0xB0E9; // TX_DATA00
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_08 = 0xB0E9; // TX_DATA01
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_09 = 0xB0E9; // TX_EN
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_10 = 0x31;   // REF_CLK
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = 0xB0E9; // RX_ER

    // Step 4: Reset PHY
    pinMode(9, OUTPUT);
    digitalWriteFast(9, LOW);
    delay(10);
    digitalWriteFast(9, HIGH);
    delay(50);

    // Step 5: Enable ENET clock
    CCM_CCGR1 |= CCM_CCGR1_ENET(CCM_CCGR_ON);

    Serial.print("\r\n  - Ethernet hardware initialization complete");
}

// HTTP event handler for GUI
void httpEventHandler(struct mg_connection *c, int ev, void *ev_data, NetworkManager *self)
{
    if (ev == MG_EV_HTTP_MSG && self)
    {
        self->handleHTTPRequest(c, ev, ev_data);
    }
}

// PGN event handler
void pgnEventHandler(struct mg_connection *c, int ev, void *ev_data, NetworkManager *self)
{
    if (ev == MG_EV_READ && self)
    {
        self->packetsReceived_++;
        Serial.print("\r\n[PGN] Received packet");

        // Call user handler if set
        if (self->pgnHandler_)
        {
            self->pgnHandler_(c, ev, ev_data);
        }
    }
}

// RTCM event handler
void rtcmEventHandler(struct mg_connection *c, int ev, void *ev_data, NetworkManager *self)
{
    if (ev == MG_EV_READ && self)
    {
        self->packetsReceived_++;
        Serial.print("\r\n[RTCM] Received packet");

        // Call user handler if set
        if (self->rtcmHandler_)
        {
            self->rtcmHandler_(c, ev, ev_data);
        }
    }
}

// AgIO event handler
void agioEventHandler(struct mg_connection *c, int ev, void *ev_data, NetworkManager *self)
{
    if (ev == MG_EV_CONNECT && self)
    {
        Serial.print("\r\n[AGIO] Connected to AgIO");
        self->lastAgioConnection_ = millis();
    }
}

NetworkManager::NetworkManager(ConfigManager &configManager, DiagnosticManager &diagnosticManager)
    : configManager_(configManager), diagnosticManager_(diagnosticManager), networkReady_(false), pgnListener_(nullptr), rtcmListener_(nullptr), agioSender_(nullptr), httpListener_(nullptr), packetsSent_(0), packetsReceived_(0), lastAgioConnection_(0), pgnHandler_(nullptr), rtcmHandler_(nullptr)
{
    Serial.print("\r\n- Network Manager constructor");
}

NetworkManager::~NetworkManager()
{
    if (networkReady_)
    {
        mg_mgr_free(&mgr);
    }
}

bool NetworkManager::begin()
{
    Serial.print("\r\n- Network Manager initialization");
    Serial.flush();

    Serial.print("\r\n- Step 1: Ethernet hardware initialization");
    Serial.flush();

    // Use your proven ethernet initialization
    ethernet_init();

    Serial.print("\r\n- Step 2: Initializing Mongoose manager");
    Serial.flush();

    // Initialize Mongoose manager
    mg_mgr_init(&mgr);

    Serial.print("\r\n- Step 3: Configuring Mongoose IP stack (SAFE MODE)");
    Serial.flush();

    // Get network config from ConfigManager
    const auto &netConfig = configManager_.getNetConfig();

    // Try a simplified approach without mg_tcpip_init for now
    Serial.print("\r\n  - Skipping mg_tcpip_init() to avoid hang");
    Serial.print("\r\n  - Using basic Mongoose initialization only");

    // Convert IP to string for display
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d",
             netConfig.currentIP[0], netConfig.currentIP[1],
             netConfig.currentIP[2], netConfig.currentIP[3]);

    char gatewayStr[16];
    snprintf(gatewayStr, sizeof(gatewayStr), "%d.%d.%d.%d",
             netConfig.gatewayIP[0], netConfig.gatewayIP[1],
             netConfig.gatewayIP[2], netConfig.gatewayIP[3]);

    Serial.print("\r\n  - Target IP: ");
    Serial.print(ipStr);
    Serial.print(" Gateway: ");
    Serial.print(gatewayStr);

    Serial.print("\r\n- Step 4: Creating UDP listeners (basic mode)");
    Serial.flush();

    // Create PGN listener on port 8888
    pgnListener_ = mg_listen(&mgr, "udp://:8888", [](struct mg_connection *c, int ev, void *ev_data)
                             {
            NetworkManager* self = static_cast<NetworkManager*>(c->fn_data);
            pgnEventHandler(c, ev, ev_data, self); }, this);

    if (pgnListener_)
    {
        Serial.print("\r\n  - PGN listener created (port 8888)");
    }
    else
    {
        Serial.print("\r\n  ERROR: Failed to create PGN listener");
    }

    // Create RTCM listener on port 2233
    rtcmListener_ = mg_listen(&mgr, "udp://:2233", [](struct mg_connection *c, int ev, void *ev_data)
                              {
            NetworkManager* self = static_cast<NetworkManager*>(c->fn_data);
            rtcmEventHandler(c, ev, ev_data, self); }, this);

    if (rtcmListener_)
    {
        Serial.print("\r\n  - RTCM listener created (port 2233)");
    }
    else
    {
        Serial.print("\r\n  ERROR: Failed to create RTCM listener");
    }

    Serial.print("\r\n- Step 5: Creating HTTP server for GUI (basic mode)");
    Serial.flush();

    // Create HTTP server on port 80 for GUI
    httpListener_ = mg_listen(&mgr, "http://:80", [](struct mg_connection *c, int ev, void *ev_data)
                              {
            NetworkManager* self = static_cast<NetworkManager*>(c->fn_data);
            httpEventHandler(c, ev, ev_data, self); }, this);

    if (httpListener_)
    {
        Serial.print("\r\n  - HTTP server created (port 80) for GUI");
    }
    else
    {
        Serial.print("\r\n  ERROR: Failed to create HTTP server");
    }

    Serial.print("\r\n- Step 6: Creating AgIO sender connection");
    Serial.flush();

    // Create AgIO sender connection to broadcast address on port 9999
    char agioURL[32];
    snprintf(agioURL, sizeof(agioURL), "udp://%d.%d.%d.255:9999",
             netConfig.broadcastIP[0], netConfig.broadcastIP[1], netConfig.broadcastIP[2]);

    agioSender_ = mg_connect(&mgr, agioURL, [](struct mg_connection *c, int ev, void *ev_data)
                             {
            NetworkManager* self = static_cast<NetworkManager*>(c->fn_data);
            agioEventHandler(c, ev, ev_data, self); }, this);

    if (agioSender_)
    {
        Serial.print("\r\n  - AgIO sender ready (port 9999)");
    }
    else
    {
        Serial.print("\r\n  ERROR: Failed to create AgIO sender");
    }

    networkReady_ = true;
    lastAgioConnection_ = millis();

    Serial.print("\r\n- SAFE MODE INITIALIZATION COMPLETE");
    Serial.print("\r\n  - Mongoose manager initialized");
    Serial.print("\r\n  - Hardware ethernet initialized");
    Serial.print("\r\n  - Listeners created (may need proper IP config)");
    Serial.print("\r\n  - Target IP: ");
    Serial.print(ipStr);
    Serial.print("\r\n  - Status: Basic mode (no TCP/IP stack init)");

    Serial.print("\r\n- NOTE: Network may not respond until proper IP configuration");
    Serial.print("\r\n  Use debug commands to test and configure");

    return true;
}

void NetworkManager::update()
{
    if (!networkReady_)
        return;

    uint32_t startTime = micros();

    // Use Mongoose's standard polling function
    mg_mgr_poll(&mgr, 0);

    // Update CPU usage tracking
    uint32_t elapsedTime = micros() - startTime;
    (void)elapsedTime; // Suppress unused variable warning

    // Check AgIO connection timeout
    if (millis() - lastAgioConnection_ > 5000)
    {
        // Connection timeout - could reconnect here if needed
    }
}

void NetworkManager::handleHTTPRequest(struct mg_connection *c, int ev, void *ev_data)
{
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;

    // Basic routing for GUI - use simple string comparison with correct mg_str member
    if (hm->uri.len == 1 && strncmp(hm->uri.buf, "/", 1) == 0)
    {
        // Serve main page
        mg_http_reply(c, 200, "Content-Type: text/html\r\n",
                      "<!DOCTYPE html><html><head><title>AgOpenGPS Control</title></head>"
                      "<body><h1>AgOpenGPS Teensy 4.1 Control Panel</h1>"
                      "<p>Status: Running</p>"
                      "<p>Packets Sent: %u</p>"
                      "<p>Packets Received: %u</p>"
                      "</body></html>",
                      packetsSent_, packetsReceived_);
    }
    else if (hm->uri.len == 11 && strncmp(hm->uri.buf, "/api/status", 11) == 0)
    {
        // API endpoint for status
        mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                      "{\"status\":\"running\",\"packetsSent\":%u,\"packetsReceived\":%u}",
                      packetsSent_, packetsReceived_);
    }
    else
    {
        // 404 for unknown URLs
        mg_http_reply(c, 404, "", "Not Found");
    }
}

void NetworkManager::serveStaticFiles(struct mg_connection *c, struct mg_http_message *hm)
{
    // Placeholder for serving static files (CSS, JS, etc.)
    mg_http_reply(c, 404, "", "Static files not implemented yet");
}

void NetworkManager::handleAPIRequests(struct mg_connection *c, struct mg_http_message *hm)
{
    // Placeholder for API requests
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"api\":\"placeholder\"}");
}

bool NetworkManager::sendPGNMessage(uint32_t pgn, const uint8_t *data, uint8_t length)
{
    if (!networkReady_ || !agioSender_ || length > 223)
    {
        return false;
    }

    // Create PGN packet header
    uint8_t packet[229]; // Max PGN packet size
    packet[0] = 0x80;
    packet[1] = 0x81;
    packet[2] = 0x7F;
    packet[3] = (uint8_t)(pgn & 0xFF);
    packet[4] = (uint8_t)((pgn >> 8) & 0xFF);
    packet[5] = (uint8_t)((pgn >> 16) & 0xFF);

    // Copy data
    memcpy(&packet[6], data, length);

    // Send packet
    mg_send(agioSender_, packet, 6 + length);
    packetsSent_++;

    return true;
}

bool NetworkManager::sendRTCMMessage(const uint8_t *data, uint16_t length)
{
    if (!networkReady_ || !agioSender_ || length > 1024)
    {
        return false;
    }

    // Send RTCM data directly
    mg_send(agioSender_, data, length);
    packetsSent_++;

    return true;
}

void NetworkManager::getNetworkStats(uint32_t &sent, uint32_t &received)
{
    sent = packetsSent_;
    received = packetsReceived_;
}

bool NetworkManager::isNetworkReady() const
{
    return networkReady_;
}