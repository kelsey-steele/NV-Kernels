/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2026 NVIDIA Corporation. All rights reserved.
 *
 * CXL register virtualization helpers for vfio-pci Type-2 passthrough.
 *
 * See Documentation/driver-api/vfio-pci-cxl.rst for the ownership
 * contract.  In short: cxl-core owns the per-device DVSEC body, HDM
 * Decoder block, and CM cap-array shadows; vfio-pci is a transport
 * that forwards guest reads and writes through the helpers below.
 *
 * The helpers are not a generic emulation framework.  Each register
 * is hand-coded against CXL r4.0 §8.1.3 and §8.2.4.20.  Adding a new
 * field is "add a case", not "add a mode".
 */
#ifndef __CXL_PASSTHROUGH_H__
#define __CXL_PASSTHROUGH_H__

#include <linux/types.h>

struct cxl_dev_state;
struct cxl_passthrough;
struct device;

/**
 * devm_cxl_passthrough_create - snapshot a Type-2 device's DVSEC + HDM +
 * CM cap-array shadows and return the opaque handle the rw helpers
 * operate on.
 *
 * @dev: device whose devres lifetime bounds the returned handle.
 * @cxlds: CXL device state with cxlds->cxl_dvsec populated and
 *	   cxlds->reg_map.resource and cxlds->reg_map.max_size describing
 *	   the component register block.  cxlds->reg_map.base is NOT
 *	   required; cxl_pci_setup_regs() releases its short-lived
 *	   ioremap before returning, so this helper takes a local
 *	   bind-time ioremap against cxlds->reg_map.resource for the
 *	   duration of the snapshot.
 *
 * On success the returned handle is bound to @dev's devres so unwind
 * happens automatically when @dev is unbound.  The handle must not be
 * freed by the caller.
 *
 * Return: a valid &struct cxl_passthrough on success, ERR_PTR(-errno)
 * on failure.
 */
struct cxl_passthrough *
devm_cxl_passthrough_create(struct device *dev, struct cxl_dev_state *cxlds);

/**
 * cxl_passthrough_dvsec_rw - read or write the CXL Device DVSEC body shadow.
 *
 * @p: handle from devm_cxl_passthrough_create().
 * @off: byte offset from the start of the DVSEC capability.  Must be
 *	 >= PCI_DVSEC_CXL_CAP and (off + sz) must lie inside the DVSEC.
 *	 Accesses to the PCI ext-cap header bytes (off < PCI_DVSEC_CXL_CAP)
 *	 are the caller's responsibility; they belong on the generic
 *	 perm-bits path, not here.
 * @val: pointer to a u32 holding the read result or the write value.
 *	 The low @sz bytes of *val are the payload; upper bytes ignored
 *	 for writes and zero for reads.
 * @sz: 1, 2, or 4.  Other values return -EINVAL.
 * @write: false for read, true for write.
 *
 * Reads serve from the shadow.  Writes update the shadow per the spec
 * attribute mode for the addressed field (LOCK is RWO, CONTROL/CONTROL2
 * are RWL gated on CONFIG_LOCK, STATUS/STATUS2 are RW1C, RANGE1/2 are
 * HwInit, Reserved/RsvdZ silently consumed).
 *
 * Known limitation: a 4-byte write whose @off straddles a 16-bit DVSEC
 * field boundary (CONTROL/STATUS at 0x0c/0x0e, CONTROL2/STATUS2 at
 * 0x10/0x12) applies only the field containing the first byte of the
 * access; the adjacent 16-bit field is not updated by the same write.
 * Standard CXL register-access patterns issue separate 2-byte accesses
 * to CONTROL, STATUS, CONTROL2 and STATUS2, so this corner case is
 * documented rather than handled.
 *
 * Return: 0 on success; -EINVAL on out-of-range or bad size.
 */
int cxl_passthrough_dvsec_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			     size_t sz, bool write);

/**
 * cxl_passthrough_hdm_rw - read or write the HDM Decoder block shadow.
 *
 * @p: handle from devm_cxl_passthrough_create().
 * @off: byte offset from the HDM block base; must be 4-byte aligned and
 *	 (off + 4) <= hdm_reg_size.  Sub-dword access is not supported on
 *	 HDM registers per CXL r4.0 §8.2.4.
 * @val: pointer to a u32 holding the read result or the write value.
 * @write: false for read, true for write.
 *
 * Reads serve from the shadow.  Writes implement the per-decoder
 * COMMIT/COMMITTED handshake (CTRL) and the RWL gating on BASE/SIZE
 * imposed by COMMITTED|LOCK_ON_COMMIT.  GLOBAL_CTRL is RW; the cap
 * header is HwInit (writes dropped); other offsets in the per-decoder
 * stride are RW shadow.
 *
 * Return: 0 on success; -EINVAL on misalignment or out-of-range.
 */
int cxl_passthrough_hdm_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			   bool write);

/**
 * cxl_passthrough_cm_rw - read or write the CXL.cache/mem cap-array snapshot.
 *
 * @p: handle from devm_cxl_passthrough_create().
 * @off: byte offset from CXL_CM_OFFSET (the start of the CM cap-array
 *	 header in the component register block); must be 4-byte aligned
 *	 and (off + 4) <= cm_snapshot_size.
 * @val: pointer to a u32 holding the read result; ignored on write.
 * @write: false for read.  Writes to the cap-array are silently dropped
 *	   (the array headers are RO per CXL r4.0 §8.2.4); the @write
 *	   parameter is present only to keep the API symmetric with the
 *	   other rw helpers and to make the drop policy explicit at the
 *	   call site.
 *
 * Return: 0 on success; -EINVAL on misalignment or out-of-range.
 */
int cxl_passthrough_cm_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			  bool write);

#endif /* __CXL_PASSTHROUGH_H__ */
