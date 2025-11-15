#include "../components/rf24/RF24.h"

extern "C" {
#include "rf24_wrapper.h"
}

static RF24 *radio = nullptr;

esp_err_t rf24_init(int ce_pin, int csn_pin) {
    if (radio) {
        return ESP_OK;
    }

    radio = new RF24(ce_pin, csn_pin, 2000000);
    if (!radio->begin()) {
        delete radio;
        radio = nullptr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t rf24_set_channel(uint8_t channel) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->setChannel(channel);
    return ESP_OK;
}

esp_err_t rf24_set_payload_size(uint8_t size) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->setPayloadSize(size);
    return ESP_OK;
}

esp_err_t rf24_open_writing_pipe(const uint8_t *address) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->openWritingPipe(address);
    return ESP_OK;
}

esp_err_t rf24_open_reading_pipe(uint8_t number, const uint8_t *address) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->openReadingPipe(number, address);
    return ESP_OK;
}

esp_err_t rf24_start_listening(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->startListening();
    return ESP_OK;
}

esp_err_t rf24_stop_listening(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->stopListening();
    return ESP_OK;
}

esp_err_t rf24_write(const uint8_t *data, size_t length) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    bool ok = radio->write(data, length);
    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t rf24_available(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    return radio->available() ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t rf24_read(uint8_t *data, size_t length) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    if (!radio->available()) return ESP_ERR_NOT_FOUND;
    radio->read(data, length);
    return ESP_OK;
}

esp_err_t rf24_flush_tx(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->flush_tx();
    return ESP_OK;
}

esp_err_t rf24_flush_rx(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->flush_rx();
    return ESP_OK;
}

esp_err_t rf24_set_retries(uint8_t delay, uint8_t count) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->setRetries(delay, count);
    return ESP_OK;
}

esp_err_t rf24_set_data_rate(rf24_datarate_t rate) {
    if (!radio) return ESP_ERR_INVALID_STATE;

    bool result = false;
    switch (rate) {
        case RF24W_250KBPS:
            result = radio->setDataRate(RF24_250KBPS);
            break;
        case RF24W_1MBPS:
            result = radio->setDataRate(RF24_1MBPS);
            break;
        case RF24W_2MBPS:
            result = radio->setDataRate(RF24_2MBPS);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    return result ? ESP_OK : ESP_FAIL;
}

esp_err_t rf24_set_power_level(rf24_powerlevel_t level) {
    if (!radio) return ESP_ERR_INVALID_STATE;

    rf24_pa_dbm_e mapped;
    switch (level) {
        case RF24W_PA_MIN: mapped = RF24_PA_MIN; break;
        case RF24W_PA_LOW: mapped = RF24_PA_LOW; break;
        case RF24W_PA_HIGH: mapped = RF24_PA_HIGH; break;
        case RF24W_PA_MAX: mapped = RF24_PA_MAX; break;
        default: return ESP_ERR_INVALID_ARG;
    }

    radio->setPALevel(mapped);
    return ESP_OK;
}

esp_err_t rf24_power_up(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->powerUp();
    return ESP_OK;
}

esp_err_t rf24_power_down(void) {
    if (!radio) return ESP_ERR_INVALID_STATE;
    radio->powerDown();
    return ESP_OK;
}

bool rf24_is_chip_connected(void) {
    if (!radio) return false;
    return radio->isChipConnected();
}

void rf24_print_details(void) {
    if (radio) radio->printDetails();
}