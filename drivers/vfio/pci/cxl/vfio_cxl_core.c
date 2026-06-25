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

	/*
	 * The CXL Component Register block is a fixed 64 KiB area (CXL r4.0
	 * §8.2.3).  cxl_pci_setup_regs() records the remaining BAR length
	 * after the regblock offset in reg_map.max_size, which is an upper
	 * bound, not the spec-defined size.  Bail if the BAR does not have
	 * room for a full component register block at the recorded offset,
	 * and publish the spec size so the UAPI, sparse-mmap exclusion, and
	 * COMP_REGS region all agree on the same window.
	 */
	if (cxlds->reg_map.max_size < CXL_COMPONENT_REG_BLOCK_SIZE)
		return -ENXIO;

	cxl->info.hdm_count               = hdm_count;
	cxl->info.hdm_reg_offset          = hdm_off;
	cxl->info.hdm_reg_size            = hdm_size;
	cxl->info.comp_reg_bir            = bir;
	cxl->info.comp_reg_offset         = bar_off;
	cxl->info.comp_reg_size           = CXL_COMPONENT_REG_BLOCK_SIZE;
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

	/*
	 * Honour the per-device opt-out (set by vfio-pci's module
	 * parameter disable_cxl, or by a variant driver before
	 * registration).  Returning -ENODEV here makes the caller
	 * treat this device as plain vfio-pci.
	 */
	if (vdev->disable_cxl)
		return -ENODEV;

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

static int vfio_pci_cxl_register_hdm(struct vfio_pci_core_device *vdev);
static int vfio_pci_cxl_register_comp_regs(struct vfio_pci_core_device *vdev);

int vfio_pci_cxl_open(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	int rc;

	if (!cxl)
		return 0;	/* plain vfio-pci device */

	rc = vfio_pci_cxl_register_comp_regs(vdev);
	if (rc) {
		pci_warn(vdev->pdev,
			 "vfio-cxl: COMP_REGS region register failed (%d)\n",
			 rc);
		return rc;
	}

	rc = vfio_pci_cxl_register_hdm(vdev);
	if (rc) {
		pci_warn(vdev->pdev,
			 "vfio-cxl: HDM region register failed (%d)\n", rc);
		/*
		 * COMP_REGS already registered above.  vfio core does not
		 * call close_device() when open_device() returns an error,
		 * so roll back the COMP_REGS dynamic region here to avoid
		 * a leaked half-registered open state.
		 */
		vfio_pci_cxl_close(vdev);
		return rc;
	}
	return 0;
}

void vfio_pci_cxl_close(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	unsigned int i;

	if (!cxl)
		return;

	for (i = vdev->num_regions; i > 0; i--) {
		struct vfio_pci_region *r = &vdev->region[i - 1];

		if (r->data != cxl)
			break;
		if (r->ops->release)
			r->ops->release(vdev, r);
		vdev->num_regions--;
	}
}

/* ------------------------------------------------------------------ */
/* HDM region: mmappable view of the device's HPA range               */
/* ------------------------------------------------------------------ */

static vm_fault_t hdm_region_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct vfio_pci_cxl_state *cxl = vma->vm_private_data;
	unsigned long off = (vmf->address - vma->vm_start) +
			    (vma->vm_pgoff << PAGE_SHIFT);
	phys_addr_t pa;

	if (!cxl || !cxl->info.hpa_size)
		return VM_FAULT_SIGBUS;
	if (off >= cxl->info.hpa_size)
		return VM_FAULT_SIGBUS;

	pa = cxl->info.hpa_base + off;
	return vmf_insert_pfn(vma, vmf->address, PHYS_PFN(pa));
}

static const struct vm_operations_struct hdm_region_vm_ops = {
	.fault = hdm_region_fault,
};

static int hdm_region_mmap(struct vfio_pci_core_device *vdev,
			   struct vfio_pci_region *region,
			   struct vm_area_struct *vma)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	pgoff_t pgoff;
	u64 req_start, req_len;

	if (!cxl || !cxl->info.hpa_size)
		return -ENODEV;

	/*
	 * vfio_pci_core_mmap() forwards the VMA with vm_pgoff still
	 * carrying the VFIO region index in the high bits.  Mask it off
	 * so req_start is the in-region offset; also overwrite vm_pgoff
	 * with the normalised value so the fault handler computes the
	 * physical address from a clean offset.
	 */
	pgoff = vma->vm_pgoff &
		((1ULL << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1);
	req_start = (u64)pgoff << PAGE_SHIFT;
	req_len   = vma->vm_end - vma->vm_start;
	if (req_start > cxl->info.hpa_size ||
	    req_len > cxl->info.hpa_size - req_start)
		return -EINVAL;

	vma->vm_pgoff = pgoff;
	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_ops = &hdm_region_vm_ops;
	vma->vm_private_data = cxl;
	return 0;
}

static ssize_t hdm_region_rw(struct vfio_pci_core_device *vdev,
			     char __user *buf, size_t count,
			     loff_t *ppos, bool iswrite)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	void *kva;

	if (!cxl || !cxl->hdm_kva)
		return -EINVAL;
	if (pos < 0 || (u64)pos > cxl->info.hpa_size ||
	    count > cxl->info.hpa_size - (u64)pos)
		return -EINVAL;

	kva = (u8 *)cxl->hdm_kva + pos;
	if (iswrite) {
		if (copy_from_user(kva, buf, count))
			return -EFAULT;
	} else {
		if (copy_to_user(buf, kva, count))
			return -EFAULT;
	}

	*ppos += count;
	return count;
}

static void hdm_region_release(struct vfio_pci_core_device *vdev,
			       struct vfio_pci_region *region)
{
}

static const struct vfio_pci_regops vfio_pci_cxl_hdm_ops = {
	.rw	 = hdm_region_rw,
	.mmap	 = hdm_region_mmap,
	.release = hdm_region_release,
};

static int vfio_pci_cxl_register_hdm(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	u32 region_type = VFIO_REGION_TYPE_PCI_VENDOR_TYPE | PCI_VENDOR_ID_CXL;
	u32 region_flags = VFIO_REGION_INFO_FLAG_READ |
			   VFIO_REGION_INFO_FLAG_WRITE |
			   VFIO_REGION_INFO_FLAG_MMAP;
	int rc;

	rc = vfio_pci_core_register_dev_region(vdev, region_type,
					       VFIO_REGION_SUBTYPE_CXL,
					       &vfio_pci_cxl_hdm_ops,
					       cxl->info.hpa_size,
					       region_flags, cxl);
	if (rc)
		return rc;

	cxl->hdm_region_idx = VFIO_PCI_NUM_REGIONS + vdev->num_regions - 1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* COMP_REGS region: thin transport to cxl-core register helpers       */
/* ------------------------------------------------------------------ */

/*
 * COMP_REGS exposes the CXL component register sub-range of the
 * device's component BAR as a pread/pwrite-only VFIO region.  Access
 * is dword-only (4-byte aligned); sub-dword access returns -EINVAL.
 * The dispatch maps each dword to one of cxl-core's three rw helpers:
 *
 *   pos < CXL_CM_OFFSET                          → zero-fill / drop
 *   CXL_CM_OFFSET <= pos < hdm_reg_offset         → cxl_passthrough_cm_rw
 *   hdm_reg_offset <= pos < hdm_reg_offset+size   → cxl_passthrough_hdm_rw
 *   pos >= hdm_reg_offset + hdm_reg_size          → zero-fill / drop
 *
 * vfio holds no shadow buffer of its own; the per-field write
 * semantics live entirely in cxl-core.
 */
static ssize_t comp_regs_rw(struct vfio_pci_core_device *vdev,
			    char __user *buf, size_t count,
			    loff_t *ppos, bool iswrite)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	resource_size_t cm_off, hdm_start, hdm_end;
	size_t done = 0;

	if (!cxl || !cxl->cxlpt)
		return -EINVAL;
	if (pos < 0 || (u64)pos > cxl->info.comp_reg_size ||
	    count > cxl->info.comp_reg_size - (u64)pos)
		return -EINVAL;
	if (!IS_ALIGNED(pos, 4) || !IS_ALIGNED(count, 4))
		return -EINVAL;

	cm_off    = CXL_CM_OFFSET;
	hdm_start = cxl->info.hdm_reg_offset;
	hdm_end   = hdm_start + cxl->info.hdm_reg_size;

	while (done < count) {
		__le32 le = 0;
		u32 v32 = 0;
		int rc;

		if (iswrite) {
			if (copy_from_user(&le, buf + done, 4))
				return done ?: -EFAULT;
			v32 = le32_to_cpu(le);
		}

		if (pos >= cm_off && pos < hdm_start) {
			rc = cxl_passthrough_cm_rw(cxl->cxlpt,
						   (u32)(pos - cm_off),
						   &v32, iswrite);
			if (rc)
				return done ?: rc;
		} else if (pos >= hdm_start && pos < hdm_end) {
			rc = cxl_passthrough_hdm_rw(cxl->cxlpt,
						    (u32)(pos - hdm_start),
						    &v32, iswrite);
			if (rc)
				return done ?: rc;
		} else if (!iswrite) {
			v32 = 0;	/* outside modelled ranges: read 0 */
		}
		/* writes outside modelled ranges are silently dropped */

		if (!iswrite) {
			le = cpu_to_le32(v32);
			if (copy_to_user(buf + done, &le, 4))
				return done ?: -EFAULT;
		}

		pos  += 4;
		done += 4;
	}

	*ppos += done;
	return done;
}

static void comp_regs_release(struct vfio_pci_core_device *vdev,
			      struct vfio_pci_region *region)
{
}

static const struct vfio_pci_regops vfio_pci_cxl_comp_regs_ops = {
	.rw	 = comp_regs_rw,
	.release = comp_regs_release,
};

static int vfio_pci_cxl_register_comp_regs(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	u32 region_type = VFIO_REGION_TYPE_PCI_VENDOR_TYPE | PCI_VENDOR_ID_CXL;
	u32 region_flags = VFIO_REGION_INFO_FLAG_READ |
			   VFIO_REGION_INFO_FLAG_WRITE;
	int rc;

	rc = vfio_pci_core_register_dev_region(vdev, region_type,
					       VFIO_REGION_SUBTYPE_CXL_COMP_REGS,
					       &vfio_pci_cxl_comp_regs_ops,
					       cxl->info.comp_reg_size,
					       region_flags, cxl);
	if (rc)
		return rc;

	cxl->comp_reg_region_idx = VFIO_PCI_NUM_REGIONS + vdev->num_regions - 1;
	return 0;
}

/* ------------------------------------------------------------------ */
/* DVSEC config-space clipping shim                                    */
/* ------------------------------------------------------------------ */

/*
 * vfio_pci_cxl_config_boundary - clip a config-rw chunk at the DVSEC body edge
 *
 * Returns the maximum byte count the caller may pass through the
 * generic chunker without straddling the CXL Device DVSEC body
 * boundary, or SIZE_MAX when no clip is required.  Used by
 * vfio_pci_config_rw_single() so the DVSEC header bytes stay on the
 * generic perm-bits path and the body bytes reach the CXL hook.
 */
size_t vfio_pci_cxl_config_boundary(struct vfio_pci_core_device *vdev,
				    loff_t pos)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	u32 body_start, body_end;

	if (!cxl)
		return SIZE_MAX;

	body_start = cxl->info.dvsec_offset + PCI_DVSEC_CXL_CAP;
	body_end   = cxl->info.dvsec_offset + cxl->info.dvsec_size;

	if (pos < body_start)
		return body_start - pos;
	if (pos < body_end)
		return body_end - pos;
	return SIZE_MAX;
}

/*
 * vfio_pci_cxl_config_rw - forward CXL DVSEC config accesses to cxl-core
 *
 * Returns the number of bytes processed on success, -ENOENT if the
 * access lies entirely outside the CXL Device DVSEC body (caller
 * takes the standard perm-bits path), or another negative errno on
 * hard failure.  vfio_pci_config_rw_single() applies
 * vfio_pci_cxl_config_boundary() before width selection, so any
 * access that reaches here was already clipped to lie entirely inside
 * the DVSEC body.
 */
ssize_t vfio_pci_cxl_config_rw(struct vfio_pci_core_device *vdev,
			       loff_t pos, size_t count, __le32 *val,
			       bool iswrite)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	u32 dvsec_off, body_start, body_end, off;
	u32 host_val;
	int rc;

	if (!cxl || !cxl->cxlpt)
		return -ENOENT;

	dvsec_off  = cxl->info.dvsec_offset;
	body_start = dvsec_off + PCI_DVSEC_CXL_CAP;
	body_end   = dvsec_off + cxl->info.dvsec_size;

	if (pos + count <= body_start || pos >= body_end)
		return -ENOENT;
	if (WARN_ON_ONCE(pos < body_start || pos + count > body_end))
		return -EINVAL;	/* caller failed to clip at body boundary */

	off = (u32)(pos - dvsec_off);
	host_val = iswrite ? le32_to_cpu(*val) : 0;

	rc = cxl_passthrough_dvsec_rw(cxl->cxlpt, off, &host_val, count,
				      iswrite);
	if (rc)
		return rc;

	if (!iswrite)
		*val = cpu_to_le32(host_val);
	return count;
}

/* ------------------------------------------------------------------ */
/* GET_INFO / GET_REGION_INFO / mmap helpers                           */
/* ------------------------------------------------------------------ */

u8 vfio_pci_cxl_get_component_reg_bar(struct vfio_pci_core_device *vdev)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;

	return cxl ? cxl->info.comp_reg_bir : U8_MAX;
}

bool vfio_pci_cxl_get_comp_reg_range(struct vfio_pci_core_device *vdev,
				     size_t *start, size_t *end)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;

	if (!cxl || !cxl->info.comp_reg_size)
		return false;

	*start = cxl->info.comp_reg_offset;
	*end   = cxl->info.comp_reg_offset + cxl->info.comp_reg_size;
	return true;
}

bool vfio_pci_cxl_mmap_overlaps_comp_regs(struct vfio_pci_core_device *vdev,
					  u64 req_start, u64 req_len)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;

	if (!cxl || !cxl->info.comp_reg_size)
		return false;

	return req_start < cxl->info.comp_reg_offset + cxl->info.comp_reg_size &&
	       req_start + req_len > cxl->info.comp_reg_offset;
}

/*
 * vfio_pci_cxl_bar_overlaps_comp_regs - check whether a BAR-relative access
 * overlaps the CXL component register sub-range.
 *
 * Returns true when @bar is the component BAR and the [@start, @start + @len)
 * window overlaps [comp_reg_offset, comp_reg_offset + comp_reg_size).  Used
 * by the raw BAR read/write and ioeventfd paths to reject accesses that
 * would bypass the COMP_REGS region and reach the physical component
 * registers directly, sidestepping cxl-core's shadow and per-field write
 * semantics.
 */
bool vfio_pci_cxl_bar_overlaps_comp_regs(struct vfio_pci_core_device *vdev,
					 int bar, u64 start, u64 len)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;

	if (!cxl || !cxl->info.comp_reg_size || !len)
		return false;
	if (bar != cxl->info.comp_reg_bir)
		return false;

	return start < cxl->info.comp_reg_offset + cxl->info.comp_reg_size &&
	       start + len > cxl->info.comp_reg_offset;
}

int vfio_pci_cxl_get_info(struct vfio_pci_core_device *vdev,
			  struct vfio_info_cap *caps)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	struct vfio_device_info_cap_cxl cap = { };

	if (!cxl)
		return 0;

	cap.header.id      = VFIO_DEVICE_INFO_CAP_CXL;
	cap.header.version = 1;
	if (cxl->info.host_firmware_committed)
		cap.flags |= VFIO_CXL_CAP_HOST_FIRMWARE_COMMITTED;
	cap.hdm_region_idx      = cxl->hdm_region_idx;
	cap.comp_reg_region_idx = cxl->comp_reg_region_idx;
	cap.comp_reg_bar        = cxl->info.comp_reg_bir;
	cap.comp_reg_offset     = cxl->info.comp_reg_offset;
	cap.comp_reg_size       = cxl->info.comp_reg_size;

	return vfio_info_add_capability(caps, &cap.header, sizeof(cap));
}

/*
 * Build a VFIO_REGION_INFO_CAP_SPARSE_MMAP that excludes the CXL
 * component register block from the mmappable areas of the
 * component BAR.  Returns -ENOTTY when the request is not for the
 * component BAR or the component BAR is not mmappable; the caller
 * (vfio_pci_ioctl_get_region_info) then continues with the standard
 * BAR path.
 */
int vfio_pci_cxl_get_region_info(struct vfio_pci_core_device *vdev,
				 struct vfio_region_info *info,
				 struct vfio_info_cap *caps)
{
	struct vfio_pci_cxl_state *cxl = vdev->cxl;
	struct vfio_region_info_cap_sparse_mmap *sparse;
	u64 bar_len, comp_start, comp_end;
	u64 before_end, after_start;
	struct vfio_region_sparse_mmap_area areas[2];
	u32 nr_areas = 0, cap_size;
	int ret;

	if (!cxl)
		return -ENOTTY;
	if (info->index != cxl->info.comp_reg_bir)
		return -ENOTTY;
	if (!cxl->info.comp_reg_size)
		return -ENOTTY;
	if (!vdev->bar_mmap_supported[info->index])
		return -ENOTTY;

	bar_len    = pci_resource_len(vdev->pdev, info->index);
	comp_start = cxl->info.comp_reg_offset;
	comp_end   = comp_start + cxl->info.comp_reg_size;

	before_end  = round_down(comp_start, PAGE_SIZE);
	after_start = round_up(comp_end, PAGE_SIZE);

	if (before_end > 0) {
		areas[nr_areas].offset = 0;
		areas[nr_areas].size   = before_end;
		nr_areas++;
	}
	if (after_start < bar_len) {
		areas[nr_areas].offset = after_start;
		areas[nr_areas].size   = bar_len - after_start;
		nr_areas++;
	}

	info->offset = VFIO_PCI_INDEX_TO_OFFSET(info->index);
	info->size   = bar_len;
	info->flags  = VFIO_REGION_INFO_FLAG_READ |
		       VFIO_REGION_INFO_FLAG_WRITE;
	if (!nr_areas)
		return 0;

	info->flags |= VFIO_REGION_INFO_FLAG_MMAP;

	cap_size = struct_size(sparse, areas, nr_areas);
	sparse = kzalloc(cap_size, GFP_KERNEL);
	if (!sparse)
		return -ENOMEM;

	sparse->header.id      = VFIO_REGION_INFO_CAP_SPARSE_MMAP;
	sparse->header.version = 1;
	sparse->nr_areas       = nr_areas;
	memcpy(sparse->areas, areas, nr_areas * sizeof(areas[0]));

	ret = vfio_info_add_capability(caps, &sparse->header, cap_size);
	kfree(sparse);
	return ret;
}
