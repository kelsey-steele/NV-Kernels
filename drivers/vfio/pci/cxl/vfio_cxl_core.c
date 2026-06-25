// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 NVIDIA Corporation. All rights reserved.
 *
 * vfio-pci CXL Type-2 device passthrough — core entry points.
 *
 * Four lifecycle hooks are inserted into vfio-pci-core: acquire and
 * release run at PCI bind / unbind, open and close run on VFIO fd
 * open / close.  This mirrors the existing vfio_pci_zdev_* integration
 * model.
 *
 * vfio_pci_cxl_acquire() runs at PCI bind time.  It performs the CXL
 * register-locator probe and HDM decoder discovery under a brief
 * pci_enable_device_mem() / pci_disable_device() bracket, then asks
 * cxl-core to register a cxl_memdev and auto-attach the
 * firmware-committed region via devm_cxl_probe_mem().  pci_disable_device()
 * clears PCI_COMMAND_MASTER but NOT PCI_COMMAND_MEMORY (see
 * do_pci_disable_device() in drivers/pci/pci.c), so the cxl-core
 * MMIO accesses performed by devm_cxl_probe_mem() after the disable
 * still succeed even with vfio-pci's PCI enable refcount returned to
 * zero.  The refcount is re-taken cleanly by vfio_pci_core_enable()
 * at first VFIO fd open.
 *
 * Acquisition is fail-closed for confirmed-CXL devices.  Devices that
 * do not advertise a CXL Device DVSEC, and CXL devices whose
 * MEM_CAPABLE bit is clear, return -ENODEV so the caller falls back
 * to plain vfio-pci behaviour.  Any other negative errno from
 * acquire() is a confirmed-CXL probe failure (locator missing, HDM
 * not single-decoder, range-active timeout, passthrough shadow
 * snapshot failure, devm_cxl_probe_mem() refusal, HDM HPA range busy)
 * and aborts the vfio-pci bind so the guest never sees a CXL device
 * with half-initialised cxl-core state.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/range.h>
#include <linux/vfio_pci_core.h>

#include <uapi/cxl/cxl_regs.h>
#include <uapi/linux/pci_regs.h>
#include <uapi/linux/vfio.h>

#include <cxl/cxl.h>
#include <cxl/passthrough.h>
#include <cxl/pci.h>

#include "../vfio_pci_priv.h"
#include "vfio_cxl_priv.h"

MODULE_IMPORT_NS("CXL");

#define VFIO_PCI_CXL_HDM_RES_NAME	"vfio-cxl-hdm"

/* ------------------------------------------------------------------ */
/* Bind-time setup helpers                                             */
/* ------------------------------------------------------------------ */

static struct vfio_pci_cxl_state *
vfio_cxl_create_device_state(struct pci_dev *pdev, u16 dvsec)
{
	struct vfio_pci_cxl_state *cxl;
	u32 hdr1;
	u16 cap;
	int rc;

	cxl = devm_cxl_dev_state_create(&pdev->dev, CXL_DEVTYPE_DEVMEM,
					pci_get_dsn(pdev), dvsec,
					struct vfio_pci_cxl_state,
					cxlds, false);
	if (!cxl)
		return ERR_PTR(-ENOMEM);

	cxl->pdev = pdev;

	rc = pci_read_config_dword(pdev, dvsec + PCI_DVSEC_HEADER1, &hdr1);
	if (rc) {
		devm_kfree(&pdev->dev, cxl);
		return ERR_PTR(-EIO);
	}
	cxl->info.dvsec_offset = dvsec;
	cxl->info.dvsec_size   = PCI_DVSEC_HEADER1_LEN(hdr1);

	rc = pci_read_config_word(pdev, dvsec + PCI_DVSEC_CXL_CAP, &cap);
	if (rc) {
		devm_kfree(&pdev->dev, cxl);
		return ERR_PTR(-EIO);
	}
	if (!(cap & PCI_DVSEC_CXL_MEM_CAPABLE)) {
		devm_kfree(&pdev->dev, cxl);
		return ERR_PTR(-ENODEV);
	}

	return cxl;
}

static int vfio_cxl_probe_regs(struct vfio_pci_cxl_state *cxl)
{
	struct cxl_dev_state *cxlds = &cxl->cxlds;
	resource_size_t hdm_off, hdm_size, bar_off;
	u8 hdm_count, bir;
	int rc;

	if (WARN_ON_ONCE(!pci_is_enabled(cxl->pdev)))
		return -EINVAL;

	rc = cxl_pci_setup_regs(cxl->pdev, CXL_REGLOC_RBI_COMPONENT,
				&cxlds->reg_map);
	if (rc)
		return rc;

	rc = cxl_get_hdm_info(cxlds, &hdm_count, &hdm_off, &hdm_size);
	if (rc)
		return rc;
	if (hdm_count != 1) {
		pci_err(cxl->pdev,
			"vfio-cxl: hdm_count=%u, only 1 supported\n",
			hdm_count);
		return -EOPNOTSUPP;
	}

	rc = cxl_regblock_get_bar_info(&cxlds->reg_map, &bir, &bar_off);
	if (rc)
		return rc;

	cxl->info.hdm_count               = hdm_count;
	cxl->info.hdm_reg_offset          = hdm_off;
	cxl->info.hdm_reg_size            = hdm_size;
	cxl->info.comp_reg_bir            = bir;
	cxl->info.comp_reg_offset         = bar_off;
	cxl->info.comp_reg_size           = cxlds->reg_map.max_size;
	cxl->info.host_firmware_committed = true;

	/*
	 * Range-active polls a config-space bit in the CXL DVSEC, not
	 * MMIO, so it is safe inside or outside the memory-decode
	 * bracket.  Keep it here so cxlds->media_ready is set before the
	 * caller drops the PCI enable refcount.
	 */
	rc = cxl_await_range_active(cxlds);
	if (rc)
		return rc;
	cxlds->media_ready = true;
	return 0;
}

static int vfio_cxl_create_memdev(struct vfio_pci_cxl_state *cxl)
{
	struct range hpa_range;
	struct cxl_memdev *cxlmd;

	/*
	 * devm_cxl_probe_mem() runs synchronously: it registers a
	 * cxl_memdev which triggers cxl_mem_probe(), endpoint port
	 * creation, and autoregion attach.  Endpoint port probe reads
	 * HDM decoder MMIO via devm_cxl_setup_hdm(); the device must
	 * therefore still be memory-decoded.  pci_disable_device() only
	 * clears PCI_COMMAND_MASTER (not _MEMORY), so the paired enable
	 * / disable done by the caller leaves the decode bit asserted
	 * and these reads succeed even with the vfio refcount at zero.
	 */
	cxlmd = devm_cxl_probe_mem(&cxl->cxlds, &hpa_range);
	if (IS_ERR(cxlmd))
		return PTR_ERR(cxlmd);

	cxl->cxlmd          = cxlmd;
	cxl->info.hpa_base  = hpa_range.start;
	cxl->info.hpa_size  = range_len(&hpa_range);
	return 0;
}

/* ------------------------------------------------------------------ */
/* HDM HPA mapping                                                     */
/* ------------------------------------------------------------------ */

static int vfio_cxl_map_hdm(struct vfio_pci_cxl_state *cxl)
{
	phys_addr_t base = cxl->info.hpa_base;
	u64 size = cxl->info.hpa_size;

	if (!size)
		return -EINVAL;

	cxl->hdm_res = request_mem_region(base, size,
					  VFIO_PCI_CXL_HDM_RES_NAME);
	if (!cxl->hdm_res) {
		pci_err(cxl->pdev,
			"vfio-cxl: HDM HPA %pa-%llx busy; check firmware mappings\n",
			&base, size);
		return -EBUSY;
	}

	cxl->hdm_kva = memremap(base, size, MEMREMAP_WB);
	if (!cxl->hdm_kva) {
		release_mem_region(base, size);
		cxl->hdm_res = NULL;
		return -ENOMEM;
	}
	return 0;
}

static void vfio_cxl_unmap_hdm(struct vfio_pci_cxl_state *cxl)
{
	if (cxl->hdm_kva) {
		memunmap(cxl->hdm_kva);
		cxl->hdm_kva = NULL;
	}
	if (cxl->hdm_res) {
		release_mem_region(cxl->info.hpa_base, cxl->info.hpa_size);
		cxl->hdm_res = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* Lifecycle hooks                                                     */
/* ------------------------------------------------------------------ */

int vfio_pci_cxl_acquire(struct vfio_pci_core_device *vdev)
{
	struct pci_dev *pdev = vdev->pdev;
	struct vfio_pci_cxl_state *cxl;
	u16 dvsec;
	int rc;

	if (!pcie_is_cxl(pdev))
		return -ENODEV;

	dvsec = pci_find_dvsec_capability(pdev, PCI_VENDOR_ID_CXL,
					  PCI_DVSEC_CXL_DEVICE);
	if (!dvsec)
		return -ENODEV;

	cxl = vfio_cxl_create_device_state(pdev, dvsec);
	if (IS_ERR(cxl)) {
		rc = PTR_ERR(cxl);
		if (rc == -ENODEV)
			return -ENODEV;	/* MEM_CAPABLE clear: treat as non-CXL. */
		pci_warn(pdev, "vfio-cxl: state alloc failed (%d)\n", rc);
		return rc;
	}

	rc = pci_enable_device_mem(pdev);
	if (rc) {
		pci_warn(pdev, "vfio-cxl: pci_enable_device_mem failed (%d)\n",
			 rc);
		goto err_free;
	}

	rc = vfio_cxl_probe_regs(cxl);
	if (rc) {
		pci_disable_device(pdev);
		pci_warn(pdev, "vfio-cxl: register probe failed (%d)\n", rc);
		goto err_free;
	}

	/*
	 * Allocate the cxl-core passthrough handle (DVSEC/HDM/CM
	 * shadows) BEFORE devm_cxl_probe_mem() so that a -ENOMEM or
	 * snapshot -EIO here is recoverable: devm_kfree() the
	 * containing state and let devres unwind cxlds.  After
	 * devm_cxl_probe_mem() publishes the memdev, no devm_kfree() is
	 * possible because cxlmd->cxlds points into the state.
	 */
	cxl->cxlpt = devm_cxl_passthrough_create(&pdev->dev, &cxl->cxlds);
	if (IS_ERR(cxl->cxlpt)) {
		rc = PTR_ERR(cxl->cxlpt);
		cxl->cxlpt = NULL;
		pci_disable_device(pdev);
		pci_warn(pdev,
			 "vfio-cxl: passthrough shadow snapshot failed (%d)\n",
			 rc);
		goto err_free;
	}

	/*
	 * Drop the PCI enable refcount before publishing the cxl_memdev:
	 * vfio_pci_core_enable() will take a fresh refcount at first VFIO
	 * fd open.  PCI_COMMAND_MEMORY stays asserted (see file header).
	 */
	pci_disable_device(pdev);

	/*
	 * Populate the DPA partition tree on cxlds before
	 * devm_cxl_probe_mem() runs.  The endpoint port probe will try to
	 * reserve the firmware-committed HDM decoder range as a DPA
	 * resource child of cxlds->dpa_res; without an explicit
	 * cxl_set_capacity() call dpa_res is zero-sized and the
	 * reservation fails with -EBUSY (see __cxl_dpa_reserve() in
	 * drivers/cxl/core/hdm.c).  Read the decoder's SIZE from the
	 * snapshot we just took and size dpa_res to cover it.
	 */
	{
		u32 size_lo = 0, size_hi = 0;
		u64 dpa_size;

		cxl_passthrough_hdm_rw(cxl->cxlpt,
				       CXL_HDM_DECODER0_SIZE_LOW_OFFSET(0),
				       &size_lo, false);
		cxl_passthrough_hdm_rw(cxl->cxlpt,
				       CXL_HDM_DECODER0_SIZE_HIGH_OFFSET(0),
				       &size_hi, false);
		dpa_size = ((u64)size_hi << 32) | size_lo;

		rc = cxl_set_capacity(&cxl->cxlds, dpa_size);
		if (rc) {
			pci_warn(pdev,
				 "vfio-cxl: cxl_set_capacity(0x%llx) failed (%d)\n",
				 dpa_size, rc);
			goto err_free;
		}
	}

	rc = vfio_cxl_create_memdev(cxl);
	if (rc) {
		pci_warn(pdev,
			 "vfio-cxl: memdev/region creation failed (%d)\n", rc);
		goto err_free;
	}

	/*
	 * Once devm_cxl_probe_mem() has published a cxl_memdev that
	 * holds a pointer into cxl->cxlds, the state must NOT be
	 * devm_kfree'd.  A failure from vfio_cxl_map_hdm() is reported
	 * to userspace; the state stays allocated for the lifetime of
	 * the PCI device, and devres unwinds it when the pdev is
	 * removed.
	 */
	rc = vfio_cxl_map_hdm(cxl);
	if (rc) {
		pci_warn(pdev, "vfio-cxl: HDM HPA mapping failed (%d)\n", rc);
		return rc;
	}

	vdev->cxl = cxl;
	pci_info(pdev,
		 "vfio-cxl: acquired (hpa=%pa/0x%llx hdm@0x%llx/0x%llx BAR%u@0x%llx/0x%llx)\n",
		 &cxl->info.hpa_base, cxl->info.hpa_size,
		 cxl->info.hdm_reg_offset, cxl->info.hdm_reg_size,
		 cxl->info.comp_reg_bir,
		 cxl->info.comp_reg_offset, cxl->info.comp_reg_size);
	return 0;

err_free:
	devm_kfree(&pdev->dev, cxl);
	return rc;
}

void vfio_pci_cxl_release(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;

	if (cxl)
		vfio_cxl_unmap_hdm(cxl);
	vdev->cxl = NULL;
}

int vfio_pci_cxl_open(struct vfio_pci_core_device *vdev)
{
	/*
	 * Region registration (HDM, COMP_REGS) is added by the next
	 * patch in this series.  This hook exists so vfio-pci-core's
	 * fd-open path has a stable call site.
	 */
	return 0;
}

void vfio_pci_cxl_close(struct vfio_pci_core_device *vdev)
{
}
