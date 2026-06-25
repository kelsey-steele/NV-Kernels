/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2026 NVIDIA Corporation. All rights reserved. */
#ifndef __VFIO_PCI_CXL_PRIV_H__
#define __VFIO_PCI_CXL_PRIV_H__

#include <linux/pci.h>
#include <linux/vfio_pci_core.h>

#include <cxl/cxl.h>
#include <cxl/passthrough.h>

/**
 * struct vfio_pci_cxl_state - per-device CXL Type-2 passthrough state
 *
 * Anchored to a vfio-pci-core device via @vdev->cxl.  Allocated by
 * devm_cxl_dev_state_create() so its lifetime is bound to the PCI
 * device; the cxl_memdev acquired via devm_cxl_probe_mem() and the
 * cxl_passthrough handle returned by devm_cxl_passthrough_create()
 * are similarly devres-anchored.
 *
 * @cxlds:	CXL device state.  MUST be the first member (enforced by
 *		devm_cxl_dev_state_create()'s static_assert).
 * @pdev:	backpointer to the PCI device.
 * @cxlmd:	cxl_memdev acquired at PCI bind via devm_cxl_probe_mem().
 * @cxlpt:	register-virtualization handle owned by cxl-core; vfio
 *		forwards DVSEC config-space, COMP_REGS region, and HDM
 *		block accesses through this opaque pointer.  See
 *		Documentation/driver-api/vfio-pci-cxl.rst.
 * @info:	snapshot of cxl-side metadata describing the device's CXL
 *		layout.  Filled in during vfio_pci_cxl_acquire() and used
 *		by the VMM-facing helpers (CAP_CXL builder, region info,
 *		COMP_REGS dispatch boundary).
 * @hdm_region_idx, @comp_reg_region_idx: VFIO region indices.
 *		Assigned by vfio_pci_cxl_open() when the regions are
 *		registered; zero on a device whose fd has never been
 *		opened.
 * @hdm_res:	request_mem_region cookie for the HPA range.
 * @hdm_kva:	memremap(MEMREMAP_WB) mapping of the HPA range.  Used
 *		for the HDM region's pread/pwrite path.  The mmap fault
 *		handler does vmf_insert_pfn from the physical HPA so the
 *		guest gets the same backing memory the host sees.
 */
struct vfio_pci_cxl_state {
	/* MUST be first member - see devm_cxl_dev_state_create() macro. */
	struct cxl_dev_state		cxlds;

	struct pci_dev		       *pdev;
	struct cxl_memdev	       *cxlmd;
	struct cxl_passthrough	       *cxlpt;

	struct {
		u16		dvsec_offset;
		u16		dvsec_size;
		phys_addr_t	hpa_base;
		u64		hpa_size;
		u8		comp_reg_bir;
		u64		comp_reg_offset;
		u64		comp_reg_size;
		u8		hdm_count;
		u64		hdm_reg_offset;
		u64		hdm_reg_size;
		bool		host_firmware_committed;
	} info;

	u32				hdm_region_idx;
	u32				comp_reg_region_idx;
	struct resource		       *hdm_res;
	void			       *hdm_kva;
};

#endif /* __VFIO_PCI_CXL_PRIV_H__ */
