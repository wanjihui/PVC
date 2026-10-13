/* Included after wifi_spi_mutex in the original SEEKFREE driver.
 * Reference: ZhangStudyLife/CYT4BB7_Air national-2026, send_buffer/send_poll.
 * RT1064 adaptation: paired RX/TX eDMA, no polling wait, asynchronous port switch
 * and UDP auto-send spacing. Original synchronous entry points remain available for init.
 */
#include "zf_device_wifi_spi_async.h"
#include "fsl_lpspi_edma.h"
#include "fsl_dmamux.h"
#include "zf_common_interrupt.h"

#define WIFI_ASYNC_MAX_PACKET       1432u // 原1424字节MCAM分片加8字节MUDP；UDP/IP后仍小于1500字节
#define WIFI_ASYNC_PHASE_TIMEOUT_MS 200u
#define WIFI_ASYNC_TOTAL_TIMEOUT_MS 500u
#define WIFI_ASYNC_AUTO_SEND_US     3000u // Module auto-sends after 2ms; reserve a 1ms margin
#define WIFI_ASYNC_TX_CHANNEL       30u
#define WIFI_ASYNC_RX_CHANNEL       31u

/* eDMA cannot use default DTCM buffers. This section is DMA-accessible SDRAM
 * and is non-cacheable in the existing MPU/scatter configuration. The SDK
 * handle also contains software TCDs and dummy data read/written by eDMA. */
AT_SDRAM_NONCACHE_SECTION_ALIGN(static lpspi_master_edma_handle_t wifi_async_dma, 32);
AT_SDRAM_NONCACHE_SECTION_ALIGN(static edma_handle_t wifi_async_tx_dma, 32);
AT_SDRAM_NONCACHE_SECTION_ALIGN(static edma_handle_t wifi_async_rx_dma, 32);
AT_SDRAM_NONCACHE_SECTION_ALIGN(static uint8 wifi_async_tx[WIFI_ASYNC_MAX_PACKET + 4u], 32);
AT_SDRAM_NONCACHE_SECTION_ALIGN(static uint8 wifi_async_rx[WIFI_ASYNC_MAX_PACKET + 4u], 32);

typedef enum
{
    WIFI_ASYNC_CLOSE_SEND, WIFI_ASYNC_CLOSE_REPLY,
    WIFI_ASYNC_CONNECT_SEND, WIFI_ASYNC_CONNECT_REPLY,
    WIFI_ASYNC_DATA_SEND, WIFI_ASYNC_DATA_DRAIN,
    WIFI_ASYNC_AUTO_SEND_WAIT, WIFI_ASYNC_RX_POLL
} wifi_async_phase_enum;

static uint8 wifi_async_enabled, wifi_async_dma_ready, wifi_async_active;
static uint8 wifi_async_dirty, wifi_async_dma_running, wifi_async_result;
static volatile uint8 wifi_async_dma_done, wifi_async_dma_failed;
static wifi_async_phase_enum wifi_async_phase;
static uint32 wifi_async_started, wifi_async_phase_at;
static volatile uint32 wifi_async_data_cycles;
static uint16 wifi_async_length, wifi_async_clock_bytes;
static uint8 wifi_async_payload[WIFI_ASYNC_MAX_PACKET]; // Caller may reuse its buffer after submit
static char wifi_async_ip[16], wifi_async_local[6];
static char wifi_async_current_port[6], wifi_async_requested_port[6];

/* DMA completion ISR: RX completion means the final SPI bits have arrived.
 * Release GPIO CS immediately; no network polling, logging or control here. */
static void wifi_async_dma_callback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle,
                                    status_t status, void *user)
{
    (void)base; (void)handle; (void)user;
    gpio_high(WIFI_SPI_CS_PIN);
    wifi_async_dma_failed = (status != kStatus_Success);
    if (wifi_async_phase == WIFI_ASYNC_DATA_SEND) wifi_async_data_cycles = DWT->CYCCNT;
    __DMB();
    wifi_async_dma_done = 1;
}

/* Abort only our two DMA channels; never reset the shared DMA controller. */
static void wifi_async_finish(uint8 result)
{
    uint32 irq_state = interrupt_global_disable();
    if (wifi_async_dma_running) LPSPI_MasterTransferAbortEDMA(LPSPI1, &wifi_async_dma);
    gpio_high(WIFI_SPI_CS_PIN);
    wifi_async_dma_running = wifi_async_dma_done = 0;
    wifi_async_active = 0;
    wifi_async_result = result;
    if (result != WIFI_SPI_ASYNC_OK) wifi_async_dirty = 1; // Discard an unflushed module buffer next time
    wifi_spi_mutex = WIFI_SPI_IDLE;
    interrupt_global_enable(irq_state);
}

uint8 wifi_spi_async_init(const char *ip, const char *local_port, const char *initial_port)
{
    if (!ip || !local_port || !initial_port || strlen(ip) >= sizeof(wifi_async_ip) ||
        strlen(local_port) >= sizeof(wifi_async_local) || strlen(initial_port) >= sizeof(wifi_async_current_port)) return 1;
    if (wifi_async_active) wifi_async_finish(WIFI_SPI_ASYNC_CANCELLED);
    if (!wifi_async_dma_ready)
    {
        /* CSI camera uses its CSI engine; FlexIO library reserves channel 0.
         * Channels 30/31 are reserved here; do not reset other channel state. */
        CLOCK_EnableClock(kCLOCK_Dma);
        if (DMA0->ERQ & ((1u << WIFI_ASYNC_TX_CHANNEL) | (1u << WIFI_ASYNC_RX_CHANNEL))) return 1;
        DMAMUX_Init(DMAMUX);
        DMAMUX_DisableChannel(DMAMUX, WIFI_ASYNC_TX_CHANNEL);
        DMAMUX_DisableChannel(DMAMUX, WIFI_ASYNC_RX_CHANNEL);
        DMAMUX_SetSource(DMAMUX, WIFI_ASYNC_TX_CHANNEL, kDmaRequestMuxLPSPI1Tx);
        DMAMUX_SetSource(DMAMUX, WIFI_ASYNC_RX_CHANNEL, kDmaRequestMuxLPSPI1Rx);
        EDMA_CreateHandle(&wifi_async_tx_dma, DMA0, WIFI_ASYNC_TX_CHANNEL);
        EDMA_CreateHandle(&wifi_async_rx_dma, DMA0, WIFI_ASYNC_RX_CHANNEL);
        NVIC_SetPriority(DMA14_DMA30_IRQn, 3);
        NVIC_SetPriority(DMA15_DMA31_IRQn, 3);
        DMAMUX_EnableChannel(DMAMUX, WIFI_ASYNC_TX_CHANNEL);
        DMAMUX_EnableChannel(DMAMUX, WIFI_ASYNC_RX_CHANNEL);
        LPSPI_MasterTransferCreateHandleEDMA(LPSPI1, &wifi_async_dma, wifi_async_dma_callback, NULL,
                                            &wifi_async_rx_dma, &wifi_async_tx_dma);
        wifi_async_dma_ready = 1;
    }
    strcpy(wifi_async_ip, ip);
    strcpy(wifi_async_local, local_port);
    strcpy(wifi_async_current_port, initial_port);
    /* Independent of IMU init/PIT and millisecond tick quantization. Do not reset
     * CYCCNT: IMU sampling also measures intervals using this shared counter. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    wifi_async_enabled = 1;
    wifi_async_dirty = wifi_async_active = wifi_async_dma_running = 0;
    wifi_async_result = WIFI_SPI_ASYNC_OK;
    return 0;
}

uint8 wifi_spi_is_busy(void) { return wifi_async_active || wifi_spi_mutex != WIFI_SPI_IDLE; }
uint8 wifi_spi_send_result(void) { return wifi_async_result; }
void wifi_spi_send_cancel(void)
{
    if (wifi_async_active) wifi_async_finish(WIFI_SPI_ASYNC_CANCELLED);
}
void wifi_spi_refresh_socket(void)
{
    if (wifi_async_active) wifi_async_finish(WIFI_SPI_ASYNC_CANCELLED);
    wifi_async_dirty = 1;
}

uint8 wifi_spi_send_async(const uint8 *buffer, uint16 length, const char *port, uint32 now_ms)
{
    if (!wifi_async_enabled || wifi_spi_is_busy()) return 1;
    if (!buffer || !length || length > WIFI_ASYNC_MAX_PACKET || !port || !port[0] ||
        strlen(port) >= sizeof(wifi_async_requested_port)) return 2;
    memcpy(wifi_async_payload, buffer, length);
    strcpy(wifi_async_requested_port, port);
    wifi_async_length = length;
    wifi_async_phase = (wifi_async_dirty || strcmp(port, wifi_async_current_port)) ?
                        WIFI_ASYNC_CLOSE_SEND : WIFI_ASYNC_DATA_SEND;
    wifi_async_started = wifi_async_phase_at = now_ms;
    wifi_async_active = 1;
    wifi_async_result = WIFI_SPI_ASYNC_BUSY;
    wifi_spi_mutex = WIFI_SPI_BUSY;
    return 0;
}

/* Nonblocking read-only transfer; outgoing UDP auto-send windows have already finished. */
uint8 wifi_spi_receive_async(uint32 now_ms)
{
    if (!wifi_async_enabled || wifi_spi_is_busy()) return 1;
    wifi_async_phase = WIFI_ASYNC_RX_POLL;
    wifi_async_started = wifi_async_phase_at = now_ms;
    wifi_async_active = 1;
    wifi_async_result = WIFI_SPI_ASYNC_BUSY;
    wifi_spi_mutex = WIFI_SPI_BUSY;
    return 0;
}
/* Only consume the software FIFO; never invoke the original synchronous SPI reader. */
uint32 wifi_spi_read_async_fifo(uint8 *buffer, uint32 capacity)
{
    uint32 length = fifo_used(&wifi_spi_fifo);
    if (length > capacity) length = capacity;
    if (length) fifo_read_buffer(&wifi_spi_fifo,buffer,&length,FIFO_READ_AND_CLEAN);
    return length;
}

/* A ready INT starts one DMA transfer. Its header/payload are sent in one
 * transaction, with simultaneous RX draining to prevent full-duplex stalls. */
static uint8 wifi_async_start_dma(void)
{
    wifi_spi_head_struct *head = (wifi_spi_head_struct *)wifi_async_tx;
    lpspi_transfer_t transfer = {0};
    int socket_length;
    memset(wifi_async_tx, 0, sizeof(wifi_async_tx));
    memset(wifi_async_rx, 0, sizeof(wifi_async_rx));
    head->command = WIFI_SPI_DATA;
    wifi_async_clock_bytes = 4u;
    switch (wifi_async_phase)
    {
        case WIFI_ASYNC_CLOSE_SEND:
            head->command = WIFI_SPI_CLOSE_SOCKET;
            wifi_async_clock_bytes = WIFI_SPI_RECVIVE_SIZE; // Match original get_parameter request
            break;
        case WIFI_ASYNC_CONNECT_SEND:
            head->command = WIFI_SPI_SET_SOCKET_INFORMATION;
            socket_length = snprintf((char *)wifi_async_tx + 4, sizeof(wifi_async_tx) - 4,
                                      "UDP\r\n%s\r\n%s\r\n%s\r\n", wifi_async_ip,
                                      wifi_async_requested_port, wifi_async_local);
            if (socket_length <= 0 || socket_length > 40) return 1;
            head->length = (uint16)socket_length;
            wifi_async_clock_bytes += head->length;
            break;
        case WIFI_ASYNC_DATA_SEND:
            head->length = wifi_async_length;
            memcpy(wifi_async_tx + 4, wifi_async_payload, wifi_async_length);
            wifi_async_clock_bytes += wifi_async_length < WIFI_SPI_RECVIVE_SIZE ? WIFI_SPI_RECVIVE_SIZE : wifi_async_length;
            break;
        case WIFI_ASYNC_CLOSE_REPLY:
        case WIFI_ASYNC_DATA_DRAIN:
        case WIFI_ASYNC_RX_POLL:
            wifi_async_clock_bytes += WIFI_SPI_RECVIVE_SIZE;
            break;
        default: break; // Socket acknowledgement is a 4-byte header
    }
    transfer.txData = wifi_async_tx;
    transfer.rxData = wifi_async_rx;
    transfer.dataSize = wifi_async_clock_bytes;
    transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    wifi_async_dma_done = wifi_async_dma_failed = 0;
    __DMB();
    gpio_low(WIFI_SPI_CS_PIN);
    if (LPSPI_MasterTransferEDMA(LPSPI1, &wifi_async_dma, &transfer) != kStatus_Success)
    {
        gpio_high(WIFI_SPI_CS_PIN);
        return 1;
    }
    wifi_async_dma_running = 1;
    return 0;
}

/* Check a wall-time deadline and return; never wait for auto-send in place. */
static void wifi_async_complete_auto_send(void)
{
    if ((uint32)(DWT->CYCCNT - wifi_async_data_cycles) >=
        WIFI_ASYNC_AUTO_SEND_US * (system_clock / 1000000u))
        wifi_async_finish(WIFI_SPI_ASYNC_OK);
}

void wifi_spi_send_poll(uint32 now_ms)
{
    wifi_spi_head_struct *reply = (wifi_spi_head_struct *)wifi_async_rx;
    wifi_async_phase_enum next;
    if (!wifi_async_active) return;
    if ((uint32)(now_ms - wifi_async_started) >= WIFI_ASYNC_TOTAL_TIMEOUT_MS ||
        (uint32)(now_ms - wifi_async_phase_at) >= WIFI_ASYNC_PHASE_TIMEOUT_MS)
    {
        wifi_async_finish(WIFI_SPI_ASYNC_TIMEOUT);
        return;
    }
    if (wifi_async_phase == WIFI_ASYNC_AUTO_SEND_WAIT)
    {
        /* SEEKFREE UDP firmware sends automatically 2ms after the last payload.
         * A manual SEND several main-loop passes later can send an empty UDP.
         * Hold the socket/payload lock through this gap so packets cannot merge
         * and a port switch cannot redirect/drop the still-buffered payload.
         * DWT measures real elapsed time even with coarse PIT ticks or slow vision.
         * OK confirms SPI completion + auto-send window, not a remote UDP ACK. */
        wifi_async_complete_auto_send();
        return; // No delay, SPI transfer or polling wait here.
    }
    if (!wifi_async_dma_running)
    {
        if (!gpio_get_level(WIFI_SPI_INT_PIN)) return; // Module busy: yield immediately, no wait_idle
        if (wifi_async_start_dma()) wifi_async_finish(WIFI_SPI_ASYNC_DMA);
        return;
    }
    if (!wifi_async_dma_done) return; // Hardware is shifting bytes; CPU returns to vision
    __DMB();
    wifi_async_dma_running = wifi_async_dma_done = 0;
    if (wifi_async_dma_failed) { wifi_async_finish(WIFI_SPI_ASYNC_DMA); return; }
    /* Only data-bearing exchanges may carry a module RX payload. A malformed
     * length must never read beyond the received DMA buffer/FIFO packet size. */
    if ((wifi_async_phase == WIFI_ASYNC_DATA_SEND || wifi_async_phase == WIFI_ASYNC_DATA_DRAIN ||
         wifi_async_phase == WIFI_ASYNC_CLOSE_REPLY || wifi_async_phase == WIFI_ASYNC_RX_POLL) &&
        (reply->command == WIFI_SPI_REPLY_DATA_START || reply->command == WIFI_SPI_REPLY_DATA_END))
    {
        if (reply->length > WIFI_SPI_RECVIVE_SIZE || reply->length > wifi_async_clock_bytes - 4u)
        { wifi_async_finish(WIFI_SPI_ASYNC_PROTOCOL); return; }
        if (reply->length) fifo_write_buffer(&wifi_spi_fifo, wifi_async_rx + 4, reply->length);
    }
    next = wifi_async_phase;
    switch (wifi_async_phase)
    {
        case WIFI_ASYNC_CLOSE_SEND: next = WIFI_ASYNC_CLOSE_REPLY; break;
        case WIFI_ASYNC_CONNECT_SEND: next = WIFI_ASYNC_CONNECT_REPLY; break;
        case WIFI_ASYNC_DATA_SEND:
            next = reply->command == WIFI_SPI_REPLY_DATA_START ? WIFI_ASYNC_DATA_DRAIN : WIFI_ASYNC_AUTO_SEND_WAIT;
            if (reply->command != WIFI_SPI_REPLY_DATA_START && reply->command != WIFI_SPI_REPLY_DATA_END &&
                reply->command != WIFI_SPI_REPLY_OK && reply->command != WIFI_SPI_INVALID1 && reply->command != WIFI_SPI_INVALID2)
            { wifi_async_finish(WIFI_SPI_ASYNC_PROTOCOL); return; }
            break;
        case WIFI_ASYNC_CLOSE_REPLY:
        case WIFI_ASYNC_CONNECT_REPLY:
            if (reply->command != WIFI_SPI_REPLY_OK) { wifi_async_finish(WIFI_SPI_ASYNC_PROTOCOL); return; }
            if (wifi_async_phase == WIFI_ASYNC_CLOSE_REPLY) next = WIFI_ASYNC_CONNECT_SEND;
            else
            {
                strcpy(wifi_async_current_port, wifi_async_requested_port);
                wifi_async_dirty = 0;
                next = WIFI_ASYNC_DATA_SEND;
            }
            break;
        case WIFI_ASYNC_DATA_DRAIN:
            if (reply->command == WIFI_SPI_REPLY_OK || reply->command == WIFI_SPI_REPLY_DATA_END ||
                reply->command == WIFI_SPI_INVALID1 || reply->command == WIFI_SPI_INVALID2) next = WIFI_ASYNC_AUTO_SEND_WAIT;
            else if (reply->command != WIFI_SPI_REPLY_DATA_START)
            { wifi_async_finish(WIFI_SPI_ASYNC_PROTOCOL); return; }
            break;
        case WIFI_ASYNC_RX_POLL:
            if (reply->command == WIFI_SPI_REPLY_DATA_START) break;
            if (reply->command != WIFI_SPI_REPLY_OK && reply->command != WIFI_SPI_REPLY_DATA_END &&
                reply->command != WIFI_SPI_INVALID1 && reply->command != WIFI_SPI_INVALID2)
            { wifi_async_finish(WIFI_SPI_ASYNC_PROTOCOL); return; }
            wifi_async_finish(WIFI_SPI_ASYNC_OK);
            return;
        case WIFI_ASYNC_AUTO_SEND_WAIT: break; // Already handled above.
    }
    /* DATA_START drain repeats are bounded by the original phase deadline. */
    if (next != wifi_async_phase) { wifi_async_phase = next; wifi_async_phase_at = now_ms; }
    /* Consume a completed exchange and start its successor in the same poll.
     * At most one new DMA submission; INT busy yields, there is no wait loop.
     * This saves empty main-loop rounds during socket changes/RX draining. */
    if (wifi_async_phase == WIFI_ASYNC_AUTO_SEND_WAIT) wifi_async_complete_auto_send();
    else if (gpio_get_level(WIFI_SPI_INT_PIN) && wifi_async_start_dma()) wifi_async_finish(WIFI_SPI_ASYNC_DMA);
}
