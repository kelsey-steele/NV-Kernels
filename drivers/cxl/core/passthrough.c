// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 NVIDIA Corporation. All rights reserved.
 *
 * vfio-pci Type-2 device passthrough — CXL register virtualization.
 *
 * Owns the CXL spec-defined virtualization semantics for the
 *   - CXL Device DVSEC capability body  (CXL r4.0 §8.1.3)
 *   - HDM Decoder Capability block      (CXL r4.0 §8.2.4.20)
 *   - CXL.cache/mem (CM) cap-array      (CXL r4.0 §8.2.4)
 *
 * vfio-pci is the only caller.  This file is NOT a generic emulation
 * framework: every register the guest may touch has a hand-written
 * write handler against the spec.  Reads serve from a shadow
 * snapshotted at create time; writes update the shadow per the spec
 * attribute mode for that field.
 *
 * Scope: firmware-committed, single-decoder, no-interleave Type-2
 * passthrough.  Multi-decoder, interleave, and hotplug are
 * out-of-scope and rejected at create time.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <uapi/cxl/cxl_regs.h>

#include <cxlpci.h>
#include <cxlmem.h>
#include <cxl/cxl.h>
#include <cxl/passthrough.h>

#include "core.h"

/* DVSEC CXL Device body offsets — relative to DVSEC capability start.
 * Body begins at PCI_DVSEC_CXL_CAP (0x0a); preceding bytes are the PCI
 * ext-cap header and DVSEC headers handled by the generic vfio
 * perm-bits path.
 */
#define DVSEC_OFF_CAPABILITY		PCI_DVSEC_CXL_CAP	/* 0x0a, u16 */
#define DVSEC_OFF_CONTROL		PCI_DVSEC_CXL_CTRL	/* 0x0c, u16 */
#define DVSEC_OFF_STATUS		0x0e			/* u16 */
#define DVSEC_OFF_CONTROL2		0x10			/* u16 */
#define DVSEC_OFF_STATUS2		0x12			/* u16 */
#define DVSEC_OFF_LOCK			0x14			/* u16 */
#define DVSEC_OFF_RANGE1_SIZE_HI	0x18			/* u32 */
#define DVSEC_OFF_RANGE1_SIZE_LO	0x1c
#define DVSEC_OFF_RANGE1_BASE_HI	0x20
#define DVSEC_OFF_RANGE1_BASE_LO	0x24
#define DVSEC_OFF_RANGE2_SIZE_HI	0x28
#define DVSEC_OFF_RANGE2_SIZE_LO	0x2c
#define DVSEC_OFF_RANGE2_BASE_HI	0x30
#define DVSEC_OFF_RANGE2_BASE_LO	0x34
#define DVSEC_BODY_END			0x38

#define DVSEC_LOCK_CONFIG_LOCK		BIT(0)

/* HDM Decoder Capability block offsets — relative to HDM block base.
 * Decoder N register set starts at 0x10 + N * 0x20.
 */
#define HDM_OFF_CAP_HEADER		0x00
#define HDM_OFF_GLOBAL_CTRL		0x04
#define HDM_DEC_BASE			0x10
#define HDM_DEC_STRIDE			0x20
#define HDM_DEC_OFF_BASE_LO(n)		(HDM_DEC_BASE + (n) * HDM_DEC_STRIDE + 0x00)
#define HDM_DEC_OFF_BASE_HI(n)		(HDM_DEC_BASE + (n) * HDM_DEC_STRIDE + 0x04)
#define HDM_DEC_OFF_SIZE_LO(n)		(HDM_DEC_BASE + (n) * HDM_DEC_STRIDE + 0x08)
#define HDM_DEC_OFF_SIZE_HI(n)		(HDM_DEC_BASE + (n) * HDM_DEC_STRIDE + 0x0c)
#define HDM_DEC_OFF_CTRL(n)		(HDM_DEC_BASE + (n) * HDM_DEC_STRIDE + 0x10)

/* HDM Decoder CTRL bits per CXL r4.0 §8.2.4.20.5. */
#define HDM_CTRL_LOCK_ON_COMMIT		BIT(8)
#define HDM_CTRL_COMMIT			BIT(9)
#define HDM_CTRL_COMMITTED		BIT(10)
#define HDM_CTRL_ERR_NOT_COMMITTED	BIT(11)

struct cxl_passthrough {
	struct cxl_dev_state *cxlds;

	/* DVSEC body shadow.  Byte-indexed by (off - PCI_DVSEC_CXL_CAP).
	 * Allocated rounded up to a dword so dword reads at the tail
	 * never overrun.
	 */
	u8 *dvsec_shadow;
	u16 dvsec_size;			/* full DVSEC cap length, incl. headers */
	bool dvsec_config_locked;

	/* HDM block shadow.  Byte-indexed; size = hdm_reg_size. */
	u8 *hdm_shadow;
	resource_size_t hdm_reg_size;

	/* CM cap-array snapshot.  Dword-indexed by (off / 4) where off
	 * is the byte offset from CXL_CM_OFFSET.  Read-only after create.
	 */
	__le32 *cm_snapshot;
	size_t cm_snapshot_dwords;

	/* Covers dvsec_shadow + dvsec_config_locked + hdm_shadow.
	 * cm_snapshot is immutable after create; no lock needed.  Leaf-
	 * level: no entry point holding this mutex calls into cxl-bus or
	 * vfio.
	 */
	struct mutex lock;
};

/* ------------------------------------------------------------------ */
/* Snapshot helpers                                                    */
/* ------------------------------------------------------------------ */

/* Read the DVSEC body bytes [PCI_DVSEC_CXL_CAP, dvsec_size) from PCI
 * config space into the shadow.
 *
 * The body starts at PCI_DVSEC_CXL_CAP (0x0a), which is word-aligned but
 * NOT dword-aligned, and CXL r4.0 §8.1.3 places six 16-bit descriptors
 * (CAPABILITY through LOCK) at offsets 0x0a..0x14 before any 32-bit
 * field.  Strict-alignment PCIe host bridges (e.g. ARM64 ECAM) reject
 * misaligned dword config accesses with PCIBIOS_BAD_REGISTER_NUMBER;
 * snapshot at the natural granularity of the body's 16-bit descriptors
 * (2-byte stride) so every offset in the range is naturally aligned.
 */
static int snapshot_dvsec_body(struct cxl_passthrough *p)
{
	struct pci_dev *pdev = to_pci_dev(p->cxlds->dev);
	u16 dvsec = p->cxlds->cxl_dvsec;
	u16 off;
	u16 word;
	int rc;

	for (off = PCI_DVSEC_CXL_CAP; off < p->dvsec_size; off += 2) {
		rc = pci_read_config_word(pdev, dvsec + off, &word);
		if (rc)
			return -EIO;
		put_unaligned_le16(word, p->dvsec_shadow +
				   (off - PCI_DVSEC_CXL_CAP));
	}
	return 0;
}

/* Read the CM cap-array prefix [CXL_CM_OFFSET, hdm_reg_offset) from
 * MMIO into cm_snapshot, and the HDM block [hdm_reg_offset,
 * hdm_reg_offset + hdm_reg_size) into hdm_shadow.
 *
 * @base is a short-lived kva for the component register block,
 * established by the caller via ioremap() against cxlds->reg_map.resource.
 * cxl_setup_regs() drops its own ioremap (clears reg_map.base) after the
 * cap-array probe completes, so this function cannot rely on
 * cxlds->reg_map.base being valid; the caller passes a fresh mapping
 * here and releases it once snapshot data has been copied into the
 * in-memory shadows.
 */
static void snapshot_cm_and_hdm(struct cxl_passthrough *p,
				void __iomem *base,
				resource_size_t hdm_off)
{
	size_t i;

	for (i = 0; i < p->cm_snapshot_dwords; i++)
		p->cm_snapshot[i] = cpu_to_le32(readl(base + CXL_CM_OFFSET +
						      i * 4));

	for (i = 0; i < p->hdm_reg_size / 4; i++)
		put_unaligned_le32(readl(base + hdm_off + i * 4),
				   p->hdm_shadow + i * 4);
}

/* ------------------------------------------------------------------ */
/* devres                                                              */
/* ------------------------------------------------------------------ */

static void cxl_passthrough_release(struct device *dev, void *res)
{
	struct cxl_passthrough *p = *(struct cxl_passthrough **)res;

	kfree(p->dvsec_shadow);
	kfree(p->hdm_shadow);
	kfree(p->cm_snapshot);
	mutex_destroy(&p->lock);
	kfree(p);
}

struct cxl_passthrough *
devm_cxl_passthrough_create(struct device *dev, struct cxl_dev_state *cxlds)
{
	struct cxl_passthrough **dres;
	struct cxl_passthrough *p;
	struct pci_dev *pdev;
	resource_size_t hdm_off, hdm_size;
	size_t dvsec_shadow_size;
	u8 hdm_count;
	u32 hdr;
	int rc;

	/*
	 * cxl_setup_regs() releases its short-lived ioremap before returning,
	 * so reg_map.base is NULL by the time we run.  Validate the persistent
	 * fields (resource address and size) instead; the local ioremap
	 * established further below covers the snapshot reads.
	 */
	if (!dev || !cxlds || !cxlds->dev || !cxlds->cxl_dvsec ||
	    !cxlds->reg_map.resource || !cxlds->reg_map.max_size)
		return ERR_PTR(-EINVAL);

	pdev = to_pci_dev(cxlds->dev);

	rc = cxl_get_hdm_info(cxlds, &hdm_count, &hdm_off, &hdm_size);
	if (rc)
		return ERR_PTR(rc);
	if (hdm_count != 1 || !hdm_size || hdm_off <= CXL_CM_OFFSET ||
	    !IS_ALIGNED(hdm_size, 4))
		return ERR_PTR(-EOPNOTSUPP);

	p = kzalloc_obj(*p, GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	mutex_init(&p->lock);
	p->cxlds = cxlds;
	p->hdm_reg_size = hdm_size;

	/* DVSEC body length from PCI ext-cap header. */
	rc = pci_read_config_dword(pdev, cxlds->cxl_dvsec + PCI_DVSEC_HEADER1,
				   &hdr);
	if (rc) {
		rc = -EIO;
		goto err;
	}
	p->dvsec_size = PCI_DVSEC_HEADER1_LEN(hdr);
	if (p->dvsec_size < DVSEC_BODY_END) {
		rc = -EINVAL;
		goto err;
	}

	dvsec_shadow_size = round_up(p->dvsec_size - PCI_DVSEC_CXL_CAP, 4);
	p->dvsec_shadow = kzalloc(dvsec_shadow_size, GFP_KERNEL);
	if (!p->dvsec_shadow) {
		rc = -ENOMEM;
		goto err;
	}

	p->cm_snapshot_dwords = (hdm_off - CXL_CM_OFFSET) / 4;
	p->cm_snapshot = kcalloc(p->cm_snapshot_dwords, sizeof(__le32),
				 GFP_KERNEL);
	if (!p->cm_snapshot) {
		rc = -ENOMEM;
		goto err;
	}

	p->hdm_shadow = kzalloc(hdm_size, GFP_KERNEL);
	if (!p->hdm_shadow) {
		rc = -ENOMEM;
		goto err;
	}

	rc = snapshot_dvsec_body(p);
	if (rc)
		goto err;

	{
		void __iomem *base;

		/*
		 * Bind-time-only ioremap.  cxl_setup_regs() has already
		 * released the cxl-core ioremap (see comment on the entry
		 * gate).  Take a fresh, short-lived mapping for the
		 * snapshot, then release it; all subsequent reads serve
		 * from the in-memory shadows.
		 */
		base = ioremap(cxlds->reg_map.resource,
			       cxlds->reg_map.max_size);
		if (!base) {
			rc = -ENOMEM;
			goto err;
		}
		snapshot_cm_and_hdm(p, base, hdm_off);
		iounmap(base);
	}

	dres = devres_alloc(cxl_passthrough_release, sizeof(*dres),
			    GFP_KERNEL);
	if (!dres) {
		rc = -ENOMEM;
		goto err;
	}
	*dres = p;
	devres_add(dev, dres);
	return p;

err:
	kfree(p->dvsec_shadow);
	kfree(p->cm_snapshot);
	kfree(p->hdm_shadow);
	mutex_destroy(&p->lock);
	kfree(p);
	return ERR_PTR(rc);
}
EXPORT_SYMBOL_NS_GPL(devm_cxl_passthrough_create, "CXL");

/* ------------------------------------------------------------------ */
/* DVSEC write semantics                                               */
/* ------------------------------------------------------------------ */

static u16 dvsec_shadow_get_u16(struct cxl_passthrough *p, u16 off)
{
	return get_unaligned_le16(p->dvsec_shadow + (off - PCI_DVSEC_CXL_CAP));
}

static void dvsec_shadow_set_u16(struct cxl_passthrough *p, u16 off, u16 val)
{
	put_unaligned_le16(val, p->dvsec_shadow + (off - PCI_DVSEC_CXL_CAP));
}

/* Apply a write to a single DVSEC field at @off, with the field's
 * native width (2 for descriptors, 4 for RANGE entries).  @width is
 * the field's spec width; @new is the merged value to apply.  Caller
 * holds p->lock.
 */
static void dvsec_apply_write(struct cxl_passthrough *p, u16 off, size_t width,
			      u32 new)
{
	u16 cur16;

	switch (off) {
	case DVSEC_OFF_CAPABILITY:
		/* HwInit — drop. */
		return;
	case DVSEC_OFF_CONTROL:
	case DVSEC_OFF_CONTROL2:
		/* RWL — gated on CONFIG_LOCK. */
		if (p->dvsec_config_locked)
			return;
		dvsec_shadow_set_u16(p, off, (u16)new);
		return;
	case DVSEC_OFF_STATUS:
	case DVSEC_OFF_STATUS2:
		/* RW1C — clear bits where the guest wrote 1. */
		cur16 = dvsec_shadow_get_u16(p, off);
		dvsec_shadow_set_u16(p, off, cur16 & ~(u16)new);
		return;
	case DVSEC_OFF_LOCK:
		/* RWO — first 1-write latches CONFIG_LOCK; subsequent
		 * writes are ignored.
		 */
		cur16 = dvsec_shadow_get_u16(p, off);
		if (cur16 & DVSEC_LOCK_CONFIG_LOCK)
			return;
		if (new & DVSEC_LOCK_CONFIG_LOCK) {
			dvsec_shadow_set_u16(p, off,
					     cur16 | DVSEC_LOCK_CONFIG_LOCK);
			p->dvsec_config_locked = true;
		}
		return;
	case DVSEC_OFF_RANGE1_SIZE_HI:
	case DVSEC_OFF_RANGE1_SIZE_LO:
	case DVSEC_OFF_RANGE1_BASE_HI:
	case DVSEC_OFF_RANGE1_BASE_LO:
		/* HwInit — drop. */
		return;
	case DVSEC_OFF_RANGE2_SIZE_HI:
	case DVSEC_OFF_RANGE2_SIZE_LO:
	case DVSEC_OFF_RANGE2_BASE_HI:
	case DVSEC_OFF_RANGE2_BASE_LO:
		/* RsvdZ — drop. */
		return;
	default:
		/* Reserved offsets inside the modelled body: drop. */
		(void)width;
		return;
	}
}

/* Map a byte offset @off inside the DVSEC body to the natural-width
 * field that contains it: returns the field's base offset (16-bit
 * aligned for descriptors, 32-bit aligned for RANGE entries) and width.
 * Returns false if @off lies outside any modelled field.
 */
static bool dvsec_field_at(u16 off, u16 *field_off, size_t *width)
{
	if (off >= DVSEC_OFF_CAPABILITY && off < DVSEC_OFF_RANGE1_SIZE_HI) {
		*field_off = ALIGN_DOWN(off, 2);
		*width = 2;
		return true;
	}
	if (off >= DVSEC_OFF_RANGE1_SIZE_HI && off < DVSEC_BODY_END) {
		*field_off = ALIGN_DOWN(off, 4);
		*width = 4;
		return true;
	}
	return false;
}

int cxl_passthrough_dvsec_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			     size_t sz, bool write)
{
	u8 *shadow;
	u16 field_off;
	size_t field_width;
	u32 cur, merged;
	u32 sub_shift;
	u32 width_mask;

	if (!p || !val)
		return -EINVAL;
	if (sz != 1 && sz != 2 && sz != 4)
		return -EINVAL;
	if (off < PCI_DVSEC_CXL_CAP || off + sz > p->dvsec_size)
		return -EINVAL;

	guard(mutex)(&p->lock);

	shadow = p->dvsec_shadow + (off - PCI_DVSEC_CXL_CAP);

	if (!write) {
		switch (sz) {
		case 1:
			*val = *shadow;
			break;
		case 2:
			*val = get_unaligned_le16(shadow);
			break;
		case 4:
			*val = get_unaligned_le32(shadow);
			break;
		}
		return 0;
	}

	if (!dvsec_field_at(off, &field_off, &field_width))
		return 0;	/* outside any modelled field: drop */

	/* Read-modify-merge the field at its natural width. */
	if (field_width == 2)
		cur = dvsec_shadow_get_u16(p, field_off);
	else
		cur = get_unaligned_le32(p->dvsec_shadow +
					 (field_off - PCI_DVSEC_CXL_CAP));

	width_mask = (sz == 4) ? 0xffffffff : (sz == 2 ? 0xffff : 0xff);
	sub_shift = (off - field_off) * 8;
	merged = cur & ~(width_mask << sub_shift);
	merged |= (*val & width_mask) << sub_shift;

	dvsec_apply_write(p, field_off, field_width, merged);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_passthrough_dvsec_rw, "CXL");

/* ------------------------------------------------------------------ */
/* HDM write semantics                                                 */
/* ------------------------------------------------------------------ */

static u32 hdm_shadow_get(struct cxl_passthrough *p, u32 off)
{
	return get_unaligned_le32(p->hdm_shadow + off);
}

static void hdm_shadow_set(struct cxl_passthrough *p, u32 off, u32 val)
{
	put_unaligned_le32(val, p->hdm_shadow + off);
}

/* Decoder index for a per-decoder register offset. */
static u32 hdm_decoder_of(u32 off)
{
	return (off - HDM_DEC_BASE) / HDM_DEC_STRIDE;
}

static u32 hdm_decoder_field(u32 off)
{
	return (off - HDM_DEC_BASE) % HDM_DEC_STRIDE;
}

static void hdm_decoder_ctrl_write(struct cxl_passthrough *p, u32 off, u32 val)
{
	u32 cur = hdm_shadow_get(p, off);
	u32 next;

	/* Once COMMITTED, only the COMMIT toggle is honoured.  Releasing
	 * COMMIT clears COMMITTED and Lock-on-Commit per CXL r4.0
	 * §8.2.4.20.5.
	 */
	if (cur & HDM_CTRL_COMMITTED) {
		next = (cur & ~HDM_CTRL_COMMIT) | (val & HDM_CTRL_COMMIT);
		if (!(val & HDM_CTRL_COMMIT)) {
			next &= ~HDM_CTRL_COMMITTED;
			next &= ~HDM_CTRL_LOCK_ON_COMMIT;
		}
		hdm_shadow_set(p, off, next);
		return;
	}

	next = val & ~(HDM_CTRL_COMMITTED | HDM_CTRL_ERR_NOT_COMMITTED);
	if (val & HDM_CTRL_COMMIT)
		next |= HDM_CTRL_COMMITTED;
	hdm_shadow_set(p, off, next);
}

static void hdm_decoder_basesize_write(struct cxl_passthrough *p, u32 off,
				       u32 val)
{
	u32 n = hdm_decoder_of(off);
	u32 ctrl = hdm_shadow_get(p, HDM_DEC_OFF_CTRL(n));

	/* RWL — BASE/SIZE locked when the decoder is committed or
	 * lock-on-commit has been latched.
	 */
	if (ctrl & (HDM_CTRL_COMMITTED | HDM_CTRL_LOCK_ON_COMMIT))
		return;
	hdm_shadow_set(p, off, val);
}

int cxl_passthrough_hdm_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			   bool write)
{
	u32 field;

	if (!p || !val)
		return -EINVAL;
	if (!IS_ALIGNED(off, 4) || off + 4 > p->hdm_reg_size)
		return -EINVAL;

	guard(mutex)(&p->lock);

	if (!write) {
		*val = hdm_shadow_get(p, off);
		return 0;
	}

	switch (off) {
	case HDM_OFF_CAP_HEADER:
		/* HwInit — drop. */
		return 0;
	case HDM_OFF_GLOBAL_CTRL:
		/* RW — shadow. */
		hdm_shadow_set(p, off, *val);
		return 0;
	}

	if (off < HDM_DEC_BASE)
		return 0;	/* gap before per-decoder regs: drop */

	field = hdm_decoder_field(off);
	switch (field) {
	case 0x00: case 0x04:	/* BASE_LO / BASE_HI */
	case 0x08: case 0x0c:	/* SIZE_LO / SIZE_HI */
		hdm_decoder_basesize_write(p, off, *val);
		return 0;
	case 0x10:		/* CTRL */
		hdm_decoder_ctrl_write(p, off, *val);
		return 0;
	default:
		/* TARGET_LIST_{LO,HI} and other per-decoder bytes are
		 * accepted as plain RW shadow for the firmware-committed
		 * scope; multi-decoder / interleave behaviour is
		 * out-of-scope.
		 */
		hdm_shadow_set(p, off, *val);
		return 0;
	}
}
EXPORT_SYMBOL_NS_GPL(cxl_passthrough_hdm_rw, "CXL");

/* ------------------------------------------------------------------ */
/* CM cap-array snapshot                                               */
/* ------------------------------------------------------------------ */

int cxl_passthrough_cm_rw(struct cxl_passthrough *p, u32 off, u32 *val,
			  bool write)
{
	if (!p || !val)
		return -EINVAL;
	if (!IS_ALIGNED(off, 4) || off / 4 >= p->cm_snapshot_dwords)
		return -EINVAL;

	if (write)
		return 0;	/* cap-array headers are RO; drop. */

	*val = le32_to_cpu(p->cm_snapshot[off / 4]);
	return 0;
}
EXPORT_SYMBOL_NS_GPL(cxl_passthrough_cm_rw, "CXL");
