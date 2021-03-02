/*
 * Copyright (c) 2017-2018 Intel Corporation
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 * See the COPYING file in the top-level directory.
 */

#ifndef HW_VIRTIO_VHOST_USER_H
#define HW_VIRTIO_VHOST_USER_H

#include "chardev/char-fe.h"
#include "hw/virtio/virtio.h"
#include "hw/pci/pci.h"
#include "hw/block/block.h"


typedef struct VhostUserHostNotifier {
    MemoryRegion mr;
    void *addr;
    bool set;
} VhostUserHostNotifier;

typedef struct VhostUserState {
    CharBackend *chr;
    VhostUserHostNotifier notifier[VIRTIO_QUEUE_MAX];
    int memory_slots;
} VhostUserState;

#include "hw/block/nvme.h"

bool vhost_user_init(VhostUserState *user, CharBackend *chr, Error **errp);
void vhost_user_cleanup(VhostUserState *user);

/* vhost user nvme */
int vhost_dev_nvme_set_backend_type(struct vhost_dev *dev,
                                    VhostBackendType backend_type);
int vhost_dev_nvme_start(struct vhost_dev *hdev, VirtIODevice *vdev);
int vhost_dev_nvme_stop(struct vhost_dev *hdev);

int vhost_user_nvme_admin_cmd_raw(struct vhost_dev *dev, NvmeCmd *cmd,
                                  void *buf, uint32_t len);
int vhost_user_nvme_get_cap(struct vhost_dev *dev, uint64_t *cap);
int vhost_user_nvme_set_bar_mr(struct vhost_dev *dev, MemoryRegion *mr);


#endif
