#ifndef HW_NVME_H
#define HW_NVME_H

#include "block/nvme.h"
#include "hw/pci/pci.h"
#include "nvme-subsys.h"
#include "nvme-ns.h"
#include "hw/virtio/vhost.h"
#include "sysemu/hostmem.h"
#include "chardev/char-fe.h"
#include "hw/pci/pci.h"

#define NVME_DEFAULT_ZONE_SIZE   (128 * MiB)
#define NVME_DEFAULT_MAX_ZA_SIZE (128 * KiB)
#define VHOST_NVME_BAR_READ 0
#define VHOST_NVME_BAR_WRITE 1

#define NVME_GUEST_ERR(trace, fmt, ...) \
    do { \
        (trace_##trace)(__VA_ARGS__); \
        qemu_log_mask(LOG_GUEST_ERROR, #trace \
            " in %s: " fmt "\n", __func__, ## __VA_ARGS__); \
    } while (0)

typedef struct NvmeParams {
    char     *serial;
    char     *vhostfd;
    uint32_t num_queues; /* deprecated since 5.1 */
    uint32_t max_ioqpairs;
    uint16_t msix_qsize;
    uint32_t cmb_size_mb;
    uint8_t  aerl;
    uint32_t aer_max_queued;
    uint8_t  mdts;
    uint8_t  vsl;
    bool     use_intel_id;
    uint8_t  zasl;
    bool     legacy_cmb;
} NvmeParams;

typedef struct NvmeAsyncEvent {
    QTAILQ_ENTRY(NvmeAsyncEvent) entry;
    NvmeAerResult result;
} NvmeAsyncEvent;

enum {
    NVME_SG_ALLOC = 1 << 0,
    NVME_SG_DMA   = 1 << 1,
};

typedef struct NvmeSg {
    int flags;

    union {
        QEMUSGList   qsg;
        QEMUIOVector iov;
    };
} NvmeSg;

typedef struct NvmeRequest {
    struct NvmeSQueue       *sq;
    struct NvmeNamespace    *ns;
    BlockAIOCB              *aiocb;
    uint16_t                status;
    void                    *opaque;
    NvmeCqe                 cqe;
    NvmeCmd                 cmd;
    BlockAcctCookie         acct;
    NvmeSg                  sg;
    QTAILQ_ENTRY(NvmeRequest)entry;
} NvmeRequest;

typedef struct NvmeBounceContext {
    NvmeRequest *req;

    struct {
        QEMUIOVector iov;
        uint8_t *bounce;
    } data, mdata;
} NvmeBounceContext;

static inline const char *nvme_adm_opc_str(uint8_t opc)
{
    switch (opc) {
    case NVME_ADM_CMD_DELETE_SQ:        return "NVME_ADM_CMD_DELETE_SQ";
    case NVME_ADM_CMD_CREATE_SQ:        return "NVME_ADM_CMD_CREATE_SQ";
    case NVME_ADM_CMD_GET_LOG_PAGE:     return "NVME_ADM_CMD_GET_LOG_PAGE";
    case NVME_ADM_CMD_DELETE_CQ:        return "NVME_ADM_CMD_DELETE_CQ";
    case NVME_ADM_CMD_CREATE_CQ:        return "NVME_ADM_CMD_CREATE_CQ";
    case NVME_ADM_CMD_IDENTIFY:         return "NVME_ADM_CMD_IDENTIFY";
    case NVME_ADM_CMD_ABORT:            return "NVME_ADM_CMD_ABORT";
    case NVME_ADM_CMD_SET_FEATURES:     return "NVME_ADM_CMD_SET_FEATURES";
    case NVME_ADM_CMD_GET_FEATURES:     return "NVME_ADM_CMD_GET_FEATURES";
    case NVME_ADM_CMD_ASYNC_EV_REQ:     return "NVME_ADM_CMD_ASYNC_EV_REQ";
    case NVME_ADM_CMD_NS_ATTACHMENT:    return "NVME_ADM_CMD_NS_ATTACHMENT";
    case NVME_ADM_CMD_FORMAT_NVM:       return "NVME_ADM_CMD_FORMAT_NVM";
    default:                            return "NVME_ADM_CMD_UNKNOWN";
    }
}

static inline const char *nvme_io_opc_str(uint8_t opc)
{
    switch (opc) {
    case NVME_CMD_FLUSH:            return "NVME_NVM_CMD_FLUSH";
    case NVME_CMD_WRITE:            return "NVME_NVM_CMD_WRITE";
    case NVME_CMD_READ:             return "NVME_NVM_CMD_READ";
    case NVME_CMD_COMPARE:          return "NVME_NVM_CMD_COMPARE";
    case NVME_CMD_WRITE_ZEROES:     return "NVME_NVM_CMD_WRITE_ZEROES";
    case NVME_CMD_DSM:              return "NVME_NVM_CMD_DSM";
    case NVME_CMD_VERIFY:           return "NVME_NVM_CMD_VERIFY";
    case NVME_CMD_COPY:             return "NVME_NVM_CMD_COPY";
    case NVME_CMD_ZONE_MGMT_SEND:   return "NVME_ZONED_CMD_MGMT_SEND";
    case NVME_CMD_ZONE_MGMT_RECV:   return "NVME_ZONED_CMD_MGMT_RECV";
    case NVME_CMD_ZONE_APPEND:      return "NVME_ZONED_CMD_ZONE_APPEND";
    default:                        return "NVME_NVM_CMD_UNKNOWN";
    }
}

typedef struct NvmeSQueue {
    struct NvmeCtrl *ctrl;
    uint16_t    sqid;
    uint16_t    cqid;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    size;
    uint64_t    dma_addr;
    QEMUTimer   *timer;
    NvmeRequest *io_req;
    QTAILQ_HEAD(, NvmeRequest) req_list;
    QTAILQ_HEAD(, NvmeRequest) out_req_list;
    QTAILQ_ENTRY(NvmeSQueue) entry;
} NvmeSQueue;

typedef struct NvmeCQueue {
    struct NvmeCtrl *ctrl;
    uint8_t     phase;
    uint16_t    cqid;
    uint16_t    irq_enabled;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    vector;
    uint32_t    size;
    uint64_t    dma_addr;
    int32_t     virq;
    EventNotifier guest_notifier;
    QEMUTimer   *timer;
    QTAILQ_HEAD(, NvmeSQueue) sq_list;
    QTAILQ_HEAD(, NvmeRequest) req_list;
} NvmeCQueue;

#define TYPE_NVME_BUS "nvme-bus"
#define NVME_BUS(obj) OBJECT_CHECK(NvmeBus, (obj), TYPE_NVME_BUS)

typedef struct NvmeBus {
    BusState parent_bus;
} NvmeBus;

#define TYPE_NVME "nvme"
#define NVME(obj) \
        OBJECT_CHECK(NvmeCtrl, (obj), TYPE_NVME)

#define TYPE_VHOST_NVME "vhost-kernel-nvme"
#define NVME_VHOST(obj) \
        OBJECT_CHECK(NvmeCtrl, (obj), TYPE_VHOST_NVME)

typedef struct NvmeFeatureVal {
    struct {
        uint16_t temp_thresh_hi;
        uint16_t temp_thresh_low;
    };
    uint32_t    async_config;
} NvmeFeatureVal;

typedef struct NvmeCtrl {
    PCIDevice    parent_obj;
    MemoryRegion bar0;
    MemoryRegion iomem;
    NvmeBar      bar;
    NvmeParams   params;
    NvmeBus      bus;

    uint16_t    cntlid;
    int32_t      bootindex;
    struct vhost_dev dev;
    uint32_t     num_io_queues;
    bool         dataplane_started;
    bool         vector_poll_started;

    bool        qs_created;
    uint32_t    page_size;
    uint16_t    page_bits;
    uint16_t    max_prp_ents;
    uint16_t    cqe_size;
    uint16_t    sqe_size;
    uint32_t    reg_size;
    uint32_t    num_namespaces;
    uint32_t    max_q_ents;
    uint8_t     outstanding_aers;
    uint32_t    irq_status;
    uint64_t    host_timestamp;                 /* Timestamp sent by the host */
    uint64_t    timestamp_set_qemu_clock_ms;    /* QEMU clock time */
    uint64_t    starttime_ms;
    uint16_t    temperature;
    uint8_t     smart_critical_warning;

    struct {
        MemoryRegion mem;
        uint8_t      *buf;
        bool         cmse;
        hwaddr       cba;
    } cmb;

    struct {
        HostMemoryBackend *dev;
        bool              cmse;
        hwaddr            cba;
    } pmr;

    uint8_t     aer_mask;
    NvmeRequest **aer_reqs;
    QTAILQ_HEAD(, NvmeAsyncEvent) aer_queue;
    int         aer_queued;

    uint32_t    dmrsl;

    /* Namespace ID is started with 1 so bitmap should be 1-based */
#define NVME_CHANGED_NSID_SIZE  (NVME_MAX_NAMESPACES + 1)
    DECLARE_BITMAP(changed_nsids, NVME_CHANGED_NSID_SIZE);

    NvmeSubsystem   *subsys;

    NvmeNamespace   namespace;
    /*
     * Attached namespaces to this controller.  If subsys is not given, all
     * namespaces in this list will always be attached.
     */
    NvmeNamespace   *namespaces[NVME_MAX_NAMESPACES];
    NvmeSQueue      **sq;
    NvmeCQueue      **cq;
    NvmeSQueue      admin_sq;
    NvmeCQueue      admin_cq;
    NvmeIdCtrl      id_ctrl;
    NvmeFeatureVal  features;
} NvmeCtrl;

struct nvme_stats {
    uint64_t units_read;
    uint64_t units_written;
    uint64_t read_commands;
    uint64_t write_commands;
};

static inline NvmeNamespace *nvme_ns(NvmeCtrl *n, uint32_t nsid)
{
    if (!nsid || nsid > n->num_namespaces) {
        return NULL;
    }

    return n->namespaces[nsid - 1];
}

static inline NvmeCQueue *nvme_cq(NvmeRequest *req)
{
    NvmeSQueue *sq = req->sq;
    NvmeCtrl *n = sq->ctrl;

    return n->cq[sq->cqid];
}

static inline NvmeCtrl *nvme_ctrl(NvmeRequest *req)
{
    NvmeSQueue *sq = req->sq;
    return sq->ctrl;
}

static inline uint16_t nvme_cid(NvmeRequest *req)
{
    if (!req) {
        return 0xffff;
    }

    return le16_to_cpu(req->cqe.cid);
}

typedef enum NvmeTxDirection {
    NVME_TX_DIRECTION_TO_DEVICE   = 0,
    NVME_TX_DIRECTION_FROM_DEVICE = 1,
} NvmeTxDirection;

void nvme_attach_ns(NvmeCtrl *n, NvmeNamespace *ns);
uint16_t nvme_bounce_data(NvmeCtrl *n, uint8_t *ptr, uint32_t len,
                          NvmeTxDirection dir, NvmeRequest *req);
uint16_t nvme_bounce_mdata(NvmeCtrl *n, uint8_t *ptr, uint32_t len,
                           NvmeTxDirection dir, NvmeRequest *req);
void nvme_rw_complete_cb(void *opaque, int ret);
uint16_t nvme_map_dptr(NvmeCtrl *n, NvmeSg *sg, size_t len,
                       NvmeCmd *cmd);
uint16_t nvme_sqid(NvmeRequest *req);
int nvme_addr_read(NvmeCtrl *n, hwaddr addr, void *buf, int size);
int nvme_check_cqid(NvmeCtrl *n, uint16_t cqid);
void nvme_inc_sq_head(NvmeSQueue *sq);
uint8_t nvme_sq_empty(NvmeSQueue *sq);
void nvme_req_clear(NvmeRequest *req);
void nvme_post_cqes(void *opaque);
void nvme_enqueue_req_completion(NvmeCQueue *cq, NvmeRequest *req);
uint16_t nvme_io_cmd(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_del_sq(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_get_log(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_del_cq(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_identify(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_abort(NvmeCtrl *n, NvmeRequest *req);
void nvme_set_timestamp(NvmeCtrl *n, uint64_t ts);
uint16_t nvme_get_feature(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_set_feature(NvmeCtrl *n, NvmeRequest *req);
uint16_t nvme_aer(NvmeCtrl *n, NvmeRequest *req);
void nvme_ctrl_reset(NvmeCtrl *n);
void nvme_process_db(NvmeCtrl *n, hwaddr addr, int val);
void nvme_check_constraints(NvmeCtrl *n, Error **errp);
void nvme_init_state(NvmeCtrl *n);
void nvme_init_cmb(NvmeCtrl *n, PCIDevice *pci_dev);
void nvme_init_pmr(NvmeCtrl *n, PCIDevice *pci_dev);
void nvme_init_ctrl(NvmeCtrl *n, PCIDevice *pci_dev);
int nvme_start_ctrl(NvmeCtrl *n);
void nvme_ctrl_shutdown(NvmeCtrl *n);

#endif /* HW_NVME_H */
