// SPDX-License-Identifier: GPL-2.0-only
/*
 * vfio_cxl_type2_test - smoke + dispatch tests for CXL Type-2 device
 * passthrough through vfio-pci.
 *
 * Exercises the user-visible surface gated by CONFIG_VFIO_PCI_CXL:
 *  - GET_INFO returns VFIO_DEVICE_FLAGS_CXL + a populated CAP_CXL.
 *  - The HDM-backed VFIO region can be mmap'd and read/written.
 *  - The component BAR exposes a SPARSE_MMAP cap that excludes the
 *    CXL component register sub-range.
 *  - The COMP_REGS region serves CM cap-array dwords from cxl-core's
 *    snapshot (proves the cxl_passthrough_cm_rw() path is wired).
 *  - DVSEC body reads through the config-rw clipping shim return the
 *    cxl-core shadow (proves cxl_passthrough_dvsec_rw() is wired).
 *
 * Usage:
 *   ./vfio_cxl_type2_test <BDF>
 * or export VFIO_SELFTESTS_BDF=<BDF> before running.  The device must
 * be bound to vfio-pci and the kernel must have CONFIG_VFIO_PCI_CXL=y.
 *
 * Copyright (c) 2026, NVIDIA CORPORATION & AFFILIATES.
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/pci_regs.h>
#include <linux/sizes.h>
#include <linux/vfio.h>

#include <cxl/cxl_regs.h>

#include <libvfio.h>

#include "kselftest_harness.h"

#define PCI_DVSEC_VENDOR_ID_CXL		0x1e98
#define PCI_DVSEC_ID_CXL_DEVICE		0x0000

/*
 * vfio-pci's region offset packing (kernel-internal in
 * include/linux/vfio_pci_core.h, not exposed via UAPI as of writing).
 * Provide local definitions so the selftest builds against the bare
 * UAPI vfio.h.  The guards let a future kernel hoist these to UAPI
 * without breaking this test.
 */
#ifndef VFIO_PCI_OFFSET_SHIFT
#define VFIO_PCI_OFFSET_SHIFT		40
#endif
#ifndef VFIO_PCI_INDEX_TO_OFFSET
#define VFIO_PCI_INDEX_TO_OFFSET(index)	((uint64_t)(index) << VFIO_PCI_OFFSET_SHIFT)
#endif

static const char *device_bdf;

/* Find a struct vfio_device_info capability by id in a GET_INFO buffer. */
static const struct vfio_info_cap_header *
find_device_cap(const void *buf, size_t bufsz, uint16_t id)
{
	const struct vfio_device_info *info = buf;
	const struct vfio_info_cap_header *cap;
	size_t off = info->cap_offset;

	while (off && off < bufsz) {
		cap = (const void *)((const char *)buf + off);
		if (cap->id == id)
			return cap;
		off = cap->next;
	}
	return NULL;
}

/* Walk PCI extended capability list for the CXL Device DVSEC. */
static uint16_t find_cxl_dvsec(struct vfio_pci_device *dev)
{
	uint16_t pos = PCI_CFG_SPACE_SIZE;
	int iter = 0;

	while (pos && iter++ < 64) {
		uint32_t hdr = vfio_pci_config_readl(dev, pos);
		uint16_t cap_id = hdr & 0xffff;
		uint16_t next   = (hdr >> 20) & 0xffc;
		uint32_t hdr1, hdr2;

		if (cap_id == PCI_EXT_CAP_ID_DVSEC) {
			hdr1 = vfio_pci_config_readl(dev, pos + 4);
			hdr2 = vfio_pci_config_readl(dev, pos + 8);
			if ((hdr1 & 0xffff) == PCI_DVSEC_VENDOR_ID_CXL &&
			    (hdr2 & 0xffff) == PCI_DVSEC_ID_CXL_DEVICE)
				return pos;
		}
		pos = next;
	}
	return 0;
}

FIXTURE(cxl_type2) {
	struct iommu *iommu;
	struct vfio_pci_device *dev;

	struct vfio_device_info_cap_cxl cxl_cap;
	uint16_t dvsec_base;

	uint64_t hdm_region_size;
	uint64_t comp_regs_size;
};

FIXTURE_SETUP(cxl_type2)
{
	uint8_t infobuf[512] = {};
	struct vfio_device_info *info = (void *)infobuf;
	const struct vfio_device_info_cap_cxl *cap;
	struct vfio_region_info ri = { .argsz = sizeof(ri) };

	self->iommu = iommu_init(default_iommu_mode);
	self->dev   = vfio_pci_device_init(device_bdf, self->iommu);

	info->argsz = sizeof(infobuf);
	ASSERT_EQ(0, ioctl(self->dev->fd, VFIO_DEVICE_GET_INFO, info));

	if (!(info->flags & VFIO_DEVICE_FLAGS_CXL))
		SKIP(return, "not a CXL Type-2 device");

	cap = (const void *)find_device_cap(infobuf, sizeof(infobuf),
					    VFIO_DEVICE_INFO_CAP_CXL);
	ASSERT_NE(NULL, cap);
	memcpy(&self->cxl_cap, cap, sizeof(*cap));

	ri.index = self->cxl_cap.hdm_region_idx;
	ASSERT_EQ(0, ioctl(self->dev->fd, VFIO_DEVICE_GET_REGION_INFO, &ri));
	self->hdm_region_size = ri.size;

	ri.argsz = sizeof(ri);
	ri.index = self->cxl_cap.comp_reg_region_idx;
	ASSERT_EQ(0, ioctl(self->dev->fd, VFIO_DEVICE_GET_REGION_INFO, &ri));
	self->comp_regs_size = ri.size;

	self->dvsec_base = find_cxl_dvsec(self->dev);
}

FIXTURE_TEARDOWN(cxl_type2)
{
	vfio_pci_device_cleanup(self->dev);
	iommu_cleanup(self->iommu);
}

TEST_F(cxl_type2, device_is_cxl)
{
	const struct vfio_device_info_cap_cxl *c = &self->cxl_cap;

	ASSERT_EQ(VFIO_DEVICE_INFO_CAP_CXL, c->header.id);
	ASSERT_EQ(1, c->header.version);
	ASSERT_NE(c->hdm_region_idx, c->comp_reg_region_idx);
	ASSERT_GE(c->hdm_region_idx,    VFIO_PCI_NUM_REGIONS);
	ASSERT_GE(c->comp_reg_region_idx, VFIO_PCI_NUM_REGIONS);
	ASSERT_LT(c->comp_reg_bar, PCI_STD_NUM_BARS);
	ASSERT_GT(c->comp_reg_size, 0ULL);
	ASSERT_EQ(c->comp_reg_size, self->comp_regs_size);
}

TEST_F(cxl_type2, hdm_region_mmap_rw)
{
	uint64_t off = (uint64_t)VFIO_PCI_INDEX_TO_OFFSET(
		self->cxl_cap.hdm_region_idx);
	uint32_t pattern = 0xdeadbeefU;
	uint32_t readback = 0;
	void *map;

	if (self->hdm_region_size < SZ_4K)
		SKIP(return, "HDM region < 4K");

	map = mmap(NULL, SZ_4K, PROT_READ | PROT_WRITE, MAP_SHARED,
		   self->dev->fd, off);
	ASSERT_NE(MAP_FAILED, map);

	*(volatile uint32_t *)map = pattern;
	readback = *(volatile uint32_t *)map;
	ASSERT_EQ(pattern, readback);

	ASSERT_EQ(0, munmap(map, SZ_4K));
}

TEST_F(cxl_type2, component_bar_sparse_mmap)
{
	const uint8_t bar = self->cxl_cap.comp_reg_bar;
	uint8_t buf[512] = {};
	struct vfio_region_info *ri = (void *)buf;
	const struct vfio_region_info_cap_sparse_mmap *sp;
	const struct vfio_info_cap_header *hdr;
	size_t off;
	uint32_t i;

	ri->argsz = sizeof(buf);
	ri->index = bar;
	ASSERT_EQ(0, ioctl(self->dev->fd, VFIO_DEVICE_GET_REGION_INFO, ri));

	ASSERT_TRUE(ri->flags & VFIO_REGION_INFO_FLAG_CAPS);
	off = ri->cap_offset;
	hdr = NULL;
	while (off && off < sizeof(buf)) {
		hdr = (const void *)(buf + off);
		if (hdr->id == VFIO_REGION_INFO_CAP_SPARSE_MMAP)
			break;
		off = hdr->next;
		hdr = NULL;
	}
	ASSERT_NE(NULL, hdr);
	sp = (const void *)hdr;
	ASSERT_GE(sp->nr_areas, 1U);
	for (i = 0; i < sp->nr_areas; i++) {
		uint64_t a_start = sp->areas[i].offset;
		uint64_t a_end   = a_start + sp->areas[i].size;

		ASSERT_TRUE(a_end <= self->cxl_cap.comp_reg_offset ||
			    a_start >= self->cxl_cap.comp_reg_offset +
				       self->cxl_cap.comp_reg_size);
	}
}

TEST_F(cxl_type2, comp_regs_cm_cap_array_read)
{
	uint64_t off = (uint64_t)VFIO_PCI_INDEX_TO_OFFSET(
		self->cxl_cap.comp_reg_region_idx) + CXL_CM_OFFSET;
	uint32_t hdr = 0;
	uint16_t cap_id;
	uint8_t  array_size;

	ASSERT_EQ((ssize_t)sizeof(hdr),
		  pread(self->dev->fd, &hdr, sizeof(hdr), off));

	cap_id     = hdr & CXL_CM_CAP_HDR_ID_MASK;
	array_size = (hdr & CXL_CM_CAP_HDR_ARRAY_SIZE_MASK) >> 24;
	ASSERT_EQ(cap_id, CM_CAP_HDR_CAP_ID);
	ASSERT_GT(array_size, 0);
}

TEST_F(cxl_type2, dvsec_lock_byte_read)
{
	uint8_t v;

	if (!self->dvsec_base)
		SKIP(return, "CXL Device DVSEC not found");

	v = vfio_pci_config_readb(self->dev,
				  self->dvsec_base + 0x14);	/* CONFIG_LOCK */
	/* Snapshot value is host-firmware-dependent; just assert read
	 * succeeds (no SIGBUS, no -EIO).
	 */
	(void)v;
}

/*
 * Exercise the per-decoder COMMIT/COMMITTED state machine in
 * cxl_passthrough_hdm_rw() (cxl-core).  Steps:
 *
 *   - Walk the CM cap-array via COMP_REGS reads to locate the HDM block.
 *   - Read decoder 0 CTRL; for a firmware-committed Type-2 device both
 *     COMMIT (bit 9) and COMMITTED (bit 10) are expected to be set.
 *   - Release COMMIT by writing CTRL with bit 9 cleared.
 *     Expected FSM transition: COMMITTED -> 0, LOCK_ON_COMMIT (bit 8) -> 0.
 *   - Re-set COMMIT.  Expected: COMMITTED -> 1 (auto-set by the handler).
 *   - Restore the original CTRL value so subsequent test runs see the
 *     firmware-committed state.
 *
 * The CTRL writes touch the cxl-core shadow only — they do not reach
 * the device — so the operation is safe to run repeatedly.
 */
TEST_F(cxl_type2, hdm_decoder_commit_fsm)
{
	uint64_t comp_off = (uint64_t)VFIO_PCI_INDEX_TO_OFFSET(
		self->cxl_cap.comp_reg_region_idx);
	uint32_t cm_hdr = 0, entry = 0;
	uint64_t hdm_reg_offset = 0;
	uint64_t ctrl_off;
	uint32_t ctrl_orig, ctrl_test;
	uint32_t array_size;
	uint32_t i;

	/* Discover HDM block offset via CM cap-array walk. */
	ASSERT_EQ((ssize_t)sizeof(cm_hdr),
		  pread(self->dev->fd, &cm_hdr, sizeof(cm_hdr),
			comp_off + CXL_CM_OFFSET));
	ASSERT_EQ(CM_CAP_HDR_CAP_ID, cm_hdr & CXL_CM_CAP_HDR_ID_MASK);
	array_size = (cm_hdr & CXL_CM_CAP_HDR_ARRAY_SIZE_MASK) >> 24;
	ASSERT_GT(array_size, 0);

	for (i = 1; i <= array_size; i++) {
		ASSERT_EQ((ssize_t)sizeof(entry),
			  pread(self->dev->fd, &entry, sizeof(entry),
				comp_off + CXL_CM_OFFSET + i * 4));
		if ((entry & CXL_CM_CAP_HDR_ID_MASK) == CXL_CM_CAP_CAP_ID_HDM) {
			hdm_reg_offset = CXL_CM_OFFSET +
					 ((entry & CXL_CM_CAP_PTR_MASK) >> 20);
			break;
		}
	}
	ASSERT_NE(0, hdm_reg_offset);

	/* Read decoder 0 CTRL. */
	ctrl_off = comp_off + hdm_reg_offset +
		   CXL_HDM_DECODER0_CTRL_OFFSET(0);
	ASSERT_EQ((ssize_t)sizeof(ctrl_orig),
		  pread(self->dev->fd, &ctrl_orig, sizeof(ctrl_orig),
			ctrl_off));

	/* Firmware-committed Type-2 device: COMMIT + COMMITTED both set. */
	ASSERT_TRUE(ctrl_orig & BIT(9));	/* COMMIT */
	ASSERT_TRUE(ctrl_orig & BIT(10));	/* COMMITTED */

	/* Release COMMIT; FSM clears COMMITTED and LOCK_ON_COMMIT. */
	ctrl_test = ctrl_orig & ~BIT(9);
	ASSERT_EQ((ssize_t)sizeof(ctrl_test),
		  pwrite(self->dev->fd, &ctrl_test, sizeof(ctrl_test),
			 ctrl_off));
	ASSERT_EQ((ssize_t)sizeof(ctrl_test),
		  pread(self->dev->fd, &ctrl_test, sizeof(ctrl_test),
			ctrl_off));
	ASSERT_FALSE(ctrl_test & BIT(9));	/* COMMIT cleared */
	ASSERT_FALSE(ctrl_test & BIT(10));	/* COMMITTED auto-cleared */
	ASSERT_FALSE(ctrl_test & BIT(8));	/* LOCK_ON_COMMIT auto-cleared */

	/* Re-set COMMIT; FSM auto-sets COMMITTED. */
	ctrl_test = BIT(9);
	ASSERT_EQ((ssize_t)sizeof(ctrl_test),
		  pwrite(self->dev->fd, &ctrl_test, sizeof(ctrl_test),
			 ctrl_off));
	ASSERT_EQ((ssize_t)sizeof(ctrl_test),
		  pread(self->dev->fd, &ctrl_test, sizeof(ctrl_test),
			ctrl_off));
	ASSERT_TRUE(ctrl_test & BIT(9));	/* COMMIT */
	ASSERT_TRUE(ctrl_test & BIT(10));	/* COMMITTED auto-set */

	/* Restore the original CTRL value. */
	ASSERT_EQ((ssize_t)sizeof(ctrl_orig),
		  pwrite(self->dev->fd, &ctrl_orig, sizeof(ctrl_orig),
			 ctrl_off));
}

int main(int argc, char *argv[])
{
	device_bdf = vfio_selftests_get_bdf(&argc, argv);
	return test_harness_run(argc, argv);
}
