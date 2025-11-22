/**
 * ==============================================================================
 * MQTT_HANDLER.H - Manual Polling Model (with Dual Logging)
 * ==============================================================================
 */

#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/mqtt.h"
#include "lwip/netif.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // <-- ADDED for printf-style functions

// *** FIX: Using your correct credentials ***
#define WIFI_SSID               "Big Family"
#define WIFI_PASSWORD           "94231233"
#define MQTT_BROKER_IP          "192.168.50.66"
#define MQTT_BROKER_PORT        1883
#define MQTT_CLIENT_ID          "pico-robot-02"
#define MQTT_PUB_PERIOD_MS      500
#define MQTT_KEEPALIVE_SEC      60
#define MQTT_RECONNECT_DELAY_MS 3000

#define MQTT_TOPIC_STATE        "robot/state"
#define MQTT_TOPIC_TELEM        "robot/telemetry"
#define MQTT_TOPIC_LOG          "robot/log" // <-- ADDED NEW LOG TOPIC

// MQTT state
static mqtt_client_t *mqtt_client = NULL;
static volatile bool mqtt_connected = false;
static ip_addr_t mqtt_ip;
static absolute_time_t next_reconnect_at;
static absolute_time_t last_successful_pub;
static uint32_t reconnect_delay_ms = MQTT_RECONNECT_DELAY_MS;
static bool reconnection_scheduled = false;

// Forward declaration for our new logging function
static inline void robot_log(const char *format, ...);
static inline void robot_mqtt_log_publish(const char *message);

// ============================================================================
// MQTT Callbacks
// ============================================================================

static void mqtt_pub_request_cb(void *arg, err_t err) {
    (void)arg;
    if (err == ERR_OK) {
        last_successful_pub = get_absolute_time();
    } else {
        // CRITICAL: Use printf here to avoid an infinite log loop
        printf("⚠ MQTT publish error: %d\n", (int)err);
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)client; 
    (void)arg;
    
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_connected = true;
        reconnection_scheduled = false;
        reconnect_delay_ms = MQTT_RECONNECT_DELAY_MS;
        last_successful_pub = get_absolute_time();
        robot_log("✓ MQTT connected\n"); // <-- CHANGED
        
        // Publish online status
        const char *online = "{\"status\":\"online\"}";
        mqtt_publish(mqtt_client, MQTT_TOPIC_STATE, online, strlen(online), 
                     1, 1, mqtt_pub_request_cb, NULL);
    } else {
        mqtt_connected = false;
        robot_log("⚠ MQTT connection failed: %d\n", status); // <-- CHANGED
        
        reconnection_scheduled = true;
        next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        if (reconnect_delay_ms < 20000) {
            reconnect_delay_ms *= 2;
        }
    }
}

// CRITICAL: Disconnect callback
static void mqtt_disconnect_cb(mqtt_client_t *client, void *arg) {
    (void)client;
    (void)arg;
    
    robot_log("⚠ MQTT disconnected!\n"); // <-- CHANGED
    mqtt_connected = false;
    reconnection_scheduled = true;
    next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
}

// ============================================================================
// WiFi Functions
// ============================================================================

static inline bool wifi_connect(void) {
    printf("Connecting to WiFi '%s'...\n", WIFI_SSID); // <-- Keep as printf (MQTT not ready)
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("⚠ WiFi connection failed\n"); // <-- Keep as printf
        return false;
    }
    
    printf("✓ WiFi connected\n"); // <-- Keep as printf
    printf("  IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list))); // <-- Keep as printf
    return true;
}

static inline void wifi_check_and_reconnect(void) {
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    
    if (link_status < 0 || link_status != CYW43_LINK_UP) {
        robot_log("⚠ WiFi link down (status=%d), reconnecting...\n", link_status); // <-- CHANGED
        mqtt_connected = false;  // Force MQTT to reconnect too
        cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 10000);
    }
}

// ============================================================================
// MQTT Functions
// ============================================================================

static inline bool mqtt_connect_to_broker(void) {
    if (!ipaddr_aton(MQTT_BROKER_IP, &mqtt_ip)) {
        printf("⚠ Invalid MQTT broker IP\n"); // <-- Keep as printf
        return false;
    }
    
    if (!mqtt_client) {
        mqtt_client = mqtt_client_new();
        if (!mqtt_client) {
            printf("⚠ Failed to create MQTT client\n"); // <-- Keep as printf
            return false;
        }
    }
    
    // CRITICAL: Set disconnect callback
    mqtt_set_inpub_callback(mqtt_client, NULL, NULL, mqtt_disconnect_cb);
    
    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id   = MQTT_CLIENT_ID;
    ci.keep_alive  = MQTT_KEEPALIVE_SEC;
    ci.will_topic  = MQTT_TOPIC_STATE;
    ci.will_msg    = "{\"status\":\"offline\"}";
    ci.will_qos    = 1;
    ci.will_retain = 1;
    
    robot_log("Connecting to MQTT broker %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT); // <-- CHANGED
    err_t err = mqtt_client_connect(mqtt_client, &mqtt_ip, MQTT_BROKER_PORT,
                                    mqtt_connection_cb, NULL, &ci);
    
    if (err != ERR_OK) {
        robot_log("⚠ MQTT connect error: %d\n", (int)err); // <-- CHANGED
        reconnection_scheduled = true;
        next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        if (reconnect_delay_ms < 20000) {
            reconnect_delay_ms *= 2;
        }
        return false;
    }
    
    return true;
}

// Check if MQTT is truly connected
static inline bool mqtt_is_connected(void) {
    if (!mqtt_client) {
        return false;
    }
    
    // Check the internal MQTT client connection state
    if (mqtt_client_is_connected(mqtt_client)) {
        return mqtt_connected;
    } else {
        // Connection dropped!
        if (mqtt_connected) {
            robot_log("⚠ MQTT connection lost (detected by client check)\n"); // <-- CHANGED
            mqtt_connected = false;
            reconnection_scheduled = true;
            next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        }
        return false;
    }
}

// ============================================================================
// Initialize MQTT with aggressive polling
// ============================================================================
static inline bool mqtt_init(void) {
    if (!mqtt_connect_to_broker()) {
        return false;
    }
    
    robot_log("Waiting for MQTT connection callback...\n"); // <-- CHANGED
    absolute_time_t end_time = make_timeout_time_ms(10000);
    
    while (absolute_time_diff_us(get_absolute_time(), end_time) > 0) {
        cyw43_arch_poll();
        
        if (mqtt_is_connected()) {
            robot_log("✓ MQTT callback received!\n"); // <-- CHANGED
            return true;
        }
        
        sleep_ms(1);
    }
    
    robot_log("⚠ MQTT connection timeout\n"); // <-- CHANGED
    return false;
}

// ============================================================================
// Reconnect if needed
// ============================================================================
static inline void mqtt_reconnect_if_needed(void) {
    // Check if we're really connected first
    if (!mqtt_is_connected()) {
        if (!reconnection_scheduled) {
            robot_log("[MQTT] Connection lost, scheduling reconnect\n"); // <-- CHANGED
            reconnection_scheduled = true;
            next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        }
        
        if (absolute_time_diff_us(get_absolute_time(), next_reconnect_at) <= 0) {
            robot_log("[MQTT] Reconnecting...\n"); // <-- CHANGED
            mqtt_connect_to_broker();
        }
    }
}

// ============================================================================
// Publish functions
// ============================================================================
static inline void robot_mqtt_publish(const char* topic, const char* payload) {
    if (!mqtt_is_connected()) {
        return;
    }
    
    err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload),
                             0, 0, mqtt_pub_request_cb, NULL);
    
    if (err != ERR_OK) {
        printf("⚠ MQTT publish failed: %d\n", (int)err); // <-- Keep as printf
        // Mark as disconnected if publish fails
        if (err == ERR_CONN || err == ERR_CLSD) {
            mqtt_connected = false;
            reconnection_scheduled = true;
            next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        }
    }
}

static inline void robot_mqtt_publish_qos(const char* topic, const char* payload, 
                                          uint8_t qos, uint8_t retain) {
    if (!mqtt_is_connected()) {
        return;
    }
    
    err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload),
                             qos, retain, mqtt_pub_request_cb, NULL);
    
    if (err != ERR_OK) {
        printf("⚠ MQTT publish failed: %d\n", (int)err); // <-- Keep as printf
        // Mark as disconnected if publish fails
        if (err == ERR_CONN || err == ERR_CLSD) {
            mqtt_connected = false;
            reconnection_scheduled = true;
            next_reconnect_at = make_timeout_time_ms(reconnect_delay_ms);
        }
    }
}

// ============================================================================
// Log publishing function (Internal - sends a pre-formatted string)
// ============================================================================
static inline void robot_mqtt_log_publish(const char *message) {
    if (!mqtt_is_connected()) {
        return; // Don't even try if not connected
    }

    // We use QoS 0 (fire and forget) for logs to avoid network congestion
    err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_LOG, message, strlen(message),
                             0, 0, mqtt_pub_request_cb, NULL);
    
    if (err != ERR_OK) {
        // Note: We use printf here to avoid an infinite logging loop
        printf("⚠ MQTT log publish failed: %d\n", (int)err);
    }
}

// ============================================================================
// NEW Dual Log Function (printf-style to Serial AND MQTT)
// ============================================================================
static inline void robot_log(const char *format, ...) {
    char log_buffer[256]; // Buffer for log messages
    
    va_list args;
    va_start(args, format);
    int len = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    va_end(args);

    if (len < 0 || len >= (int)sizeof(log_buffer)) {
        return; // Formatting error or buffer overflow
    }

    // 1. Print to serial monitor
    // We use printf, which handles the serial output
    printf("%s", log_buffer);
    
    // 2. Publish to MQTT
    robot_mqtt_log_publish(log_buffer);
}


// ============================================================================
// Heartbeat function - call this regularly in your main loop
// ============================================================================
static inline void mqtt_maintain_connection(void) {
    // Check WiFi first
    wifi_check_and_reconnect();
    
    // Always poll the network stack
    cyw43_arch_poll(); // <-- ADDED POLL BACK
    
    // Check and reconnect MQTT if needed
    mqtt_reconnect_if_needed();
}

#endif // MQTT_HANDLER_H