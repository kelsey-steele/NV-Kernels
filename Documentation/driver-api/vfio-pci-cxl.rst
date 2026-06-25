.. SPDX-License-Identifier: GPL-2.0
.. include:: <isonum.txt>

===========================================
VFIO-PCI: CXL Type-2 device passthrough
===========================================

:Author: Manish Honap <mhonap@nvidia.com>

Overview
========

vfio-pci-core, when built with ``CONFIG_VFIO_PCI_CXL=y``, passes a
CXL Type-2 accelerator (CXL r4.0, HDM-D / HDM-DB) through to a KVM
guest.  The host firmware commits the endpoint's HDM decoder before
vfio-pci binds; the guest sees a CXL Type-2 device whose CXL.mem
range is already programmed and locked.  The guest may inspect the
HDM Decoder Capability block and DVSEC Device capability via spec-
defined paths, and access the device's CXL.mem range as
mmap'd memory.

Scope
=====

The supported scope is intentionally narrow:

* One CXL endpoint per host bridge.
* The endpoint exposes exactly one HDM decoder (decoder 0).
* No interleave.
* Host firmware has committed the endpoint HDM decoder before
  vfio-pci probes.  Devices whose HDM decoder is *uncommitted* fail
  vfio-pci bind cleanly.
* The host bridge is in single-RP-passthrough mode (the CXL host
  bridge's own HDM decoder is not used; CFMWS-to-RP decode flows
  implicitly).  This assumption is currently *not enforced* by
  vfio-pci-core; it is a known limitation, see the Known
  limitations section.

Multi-decoder, interleave, FLR / reset state-machine integration,
and host-bridge HDM decoder programming are explicitly out of scope.
Adding any of them is additive on top of the contract described
below.

Driver model
============

There is no dedicated ``vfio-cxl`` PCI driver.  vfio-pci is the only
driver that binds to the host PCI device.  When built with
``CONFIG_VFIO_PCI_CXL=y``, vfio-pci-core calls into the cxl subsystem
to do four things at bind time:

1. ``devm_cxl_dev_state_create()`` — allocate per-device CXL state
   embedded in ``struct vfio_pci_cxl_state``.
2. ``cxl_pci_setup_regs()`` + ``cxl_get_hdm_info()`` — probe the
   Register Locator DVSEC and harvest the HDM block's BAR-relative
   offset and size.
3. ``cxl_await_range_active()`` — wait for the firmware-committed
   range to become live.
4. ``devm_cxl_passthrough_create()`` — snapshot the CXL Device DVSEC
   body, the HDM Decoder block, and the CXL.cache/mem cap-array
   prefix into shadows owned by cxl-core.  All subsequent
   register-virtualization happens inside ``drivers/cxl/core/passthrough.c``.
5. ``devm_cxl_probe_mem()`` — register a ``cxl_memdev``, enumerate
   the endpoint port, and auto-attach the firmware-committed
   region.  cxl_mem binds to the memdev as it would for any other
   Type-2 accelerator.

Ownership split
===============

Each device-visible surface is owned by exactly one subsystem:

============================================  ==============================================
Surface                                       Owner
============================================  ==============================================
PCI config (non-DVSEC, non-CXL)               vfio-pci-core ``vconfig`` (existing perm-bits)
CXL Device DVSEC body                         cxl-core ``cxl_passthrough_dvsec_rw()``
HDM Decoder Capability block                  cxl-core ``cxl_passthrough_hdm_rw()``
CM cap-array (read-only snapshot)             cxl-core ``cxl_passthrough_cm_rw()``
``cxl_memdev`` / endpoint port / autoregion   cxl-core ``devm_cxl_probe_mem()``
HDM HPA range mapping                         vfio-pci ``request_mem_region`` + ``memremap``
Sparse mmap layout for the component BAR      vfio-pci
============================================  ==============================================

The vfio side holds no shadow buffer of its own.  ``vfio_pci_cxl_state``
caches small scalars (DVSEC offset/size, HDM offset/size, component
BAR layout) for dispatch decisions; the actual virtualization
semantics live in cxl-core.

Bind sequence
=============

``vfio_pci_cxl_acquire()`` is called from
``vfio_pci_core_register_device()`` at PCI bind time.  The sequence::

  0. devm_cxl_dev_state_create(parent, CXL_DEVTYPE_DEVMEM, dsn,
                               dvsec_off, vfio_pci_cxl_state, cxlds,
                               /*mbox=*/false)

  1. pcie_is_cxl() and pci_find_dvsec_capability(CXL_DEVICE)
     -> -ENODEV if either is absent
     -> -ENODEV if the DVSEC's MEM_CAPABLE bit is clear

  2. pci_enable_device_mem()

     2a. cxl_pci_setup_regs(CXL_REGLOC_RBI_COMPONENT)
     2b. cxl_get_hdm_info() — REJECT hdm_count != 1 with -EOPNOTSUPP
     2c. cxl_regblock_get_bar_info()
     2d. cxl_await_range_active()
     2e. devm_cxl_passthrough_create(&pdev->dev, &cxlds)

  3. pci_disable_device()
     Clears PCI_COMMAND_MASTER but NOT PCI_COMMAND_MEMORY (see
     do_pci_disable_device() in drivers/pci/pci.c).  Subsequent
     MMIO from step 4 still succeeds.

  4. devm_cxl_probe_mem(&cxlds, &hpa_range)
     Registers the memdev, enumerates the endpoint port, attaches
     the firmware-committed autoregion.

  5. request_mem_region(hpa_base, hpa_size) + memremap_wb()

  6. vdev->cxl = cxl  (state published; HDM and COMP_REGS regions
     are registered later when the VFIO fd is opened)

Fail-closed semantics
---------------------

Three errnos are mapped to "not a CXL device; caller falls back to
plain vfio-pci": ``pcie_is_cxl()`` false, DVSEC absent, ``MEM_CAPABLE``
clear.  All three return ``-ENODEV`` from
``vfio_pci_cxl_acquire()``; the caller treats them as a silent
fall-through.

Any other negative errno from the bind sequence aborts the vfio-pci
bind entirely.  The guest never sees a half-initialised CXL device.
Once ``devm_cxl_probe_mem()`` has succeeded the published memdev
holds a pointer into the embedded ``cxl_dev_state``; a failure in
``vfio_cxl_map_hdm()`` after that point cannot ``devm_kfree(cxl)``
and leaves the state allocated for the lifetime of the PCI device
(devres unwinds it at pdev removal).

VFIO regions exposed
====================

When the VFIO fd is first opened, ``vfio_pci_cxl_open()`` registers
two additional regions on top of the standard vfio-pci BARs / config
region:

HDM region (``VFIO_REGION_SUBTYPE_CXL``)
  Mappable view of the device's firmware-committed HPA range.

  * ``mmap``: fault handler does
    ``vmf_insert_pfn(vma, addr, PHYS_PFN(hpa_base + off))``.  The
    guest gets the same backing physical memory the host sees.
  * ``pread`` / ``pwrite``: served from the ``memremap_wb()`` kva
    captured at bind time.

COMP_REGS region (``VFIO_REGION_SUBTYPE_CXL_COMP_REGS``)
  Shadow of the CXL component register sub-range.  ``pread`` /
  ``pwrite`` only; ``mmap`` is intentionally not supported (the VMM
  uses this region instead of mmapping the BAR).  Dword-aligned
  access only; sub-dword accesses return ``-EINVAL``.

  Dispatch by offset:

  ============================================  =================================
  Offset range                                  cxl-core helper
  ============================================  =================================
  ``< CXL_CM_OFFSET``                           zero-fill (reserved)
  ``CXL_CM_OFFSET .. hdm_reg_offset``           ``cxl_passthrough_cm_rw()``
  ``hdm_reg_offset .. +hdm_reg_size``           ``cxl_passthrough_hdm_rw()``
  ``>= hdm_reg_offset + hdm_reg_size``          zero-fill (reserved)
  ============================================  =================================

DVSEC virtualization contract
=============================

The CXL Device DVSEC body is reached through the standard PCI
config-space path.  ``vfio_pci_config_rw_single()`` clips chunks at
the DVSEC body boundary via ``vfio_pci_cxl_config_boundary()`` and
forwards body bytes to ``vfio_pci_cxl_config_rw()``, which in turn
calls ``cxl_passthrough_dvsec_rw()``.

Per-field write semantics (CXL r4.0 §8.1.3):

============================================  ==============================================
Field (offset from DVSEC cap base)            Spec attribute / behaviour
============================================  ==============================================
CAPABILITY        (0x0a)                      HwInit — writes dropped
CONTROL           (0x0c)                      RWL — gated on DVSEC CONFIG_LOCK
STATUS            (0x0e)                      RW1C
CONTROL2          (0x10)                      RWL — gated on DVSEC CONFIG_LOCK
STATUS2           (0x12)                      RW1C
LOCK              (0x14)                      RWO — first 1-write latches CONFIG_LOCK
Range1 SIZE_HI/LO BASE_HI/LO  (0x18..0x27)    HwInit — writes dropped
Range2 SIZE_HI/LO BASE_HI/LO  (0x28..0x37)    RsvdZ — writes dropped
============================================  ==============================================

HDM virtualization contract
===========================

Per CXL r4.0 §8.2.4.20, on the single firmware-committed decoder:

============================================  ==============================================
Field (offset from HDM block base)            Spec attribute / behaviour
============================================  ==============================================
HDM Decoder Capability Header (0x00)          HwInit — writes dropped
HDM Decoder Global Control    (0x04)          RW — shadow
Decoder 0 BASE_LO / BASE_HI                   RWL — gated on COMMITTED or LOCK_ON_COMMIT
Decoder 0 SIZE_LO / SIZE_HI                   RWL — same gate
Decoder 0 CTRL                                Implements COMMIT → COMMITTED handshake; once
                                              COMMITTED, only COMMIT toggles are honoured
============================================  ==============================================

CM cap-array
============

The CM cap-array (CXL r4.0 §8.2.4) prefix is snapshotted from the
device's component register MMIO at bind time and served read-only
through ``cxl_passthrough_cm_rw()``.  Guest writes to the cap-array
are silently dropped.

UAPI: CAP_CXL
=============

``VFIO_DEVICE_GET_INFO`` returns ``VFIO_DEVICE_FLAGS_CXL`` and a
``VFIO_DEVICE_INFO_CAP_CXL`` capability::

    struct vfio_device_info_cap_cxl {
        struct vfio_info_cap_header header;
        __u32 flags;
        #define VFIO_CXL_CAP_HOST_FIRMWARE_COMMITTED (1 << 0)
        __u32 hdm_region_idx;
        __u32 comp_reg_region_idx;
        __u32 comp_reg_bar;
        __u32 __resv;
        __u64 comp_reg_offset;
        __u64 comp_reg_size;
    };

``VFIO_DEVICE_GET_REGION_INFO`` on the component BAR returns a
``VFIO_REGION_INFO_CAP_SPARSE_MMAP`` that excludes
``[comp_reg_offset, comp_reg_offset + comp_reg_size)`` from the
mmappable areas.

Known limitations
=================

* Host bridge HDM decoder programming is not driven by this driver.
  The driver silently assumes single-RP-passthrough topology (the
  CXL host bridge's own HDM decoder is not used).  Two remediations
  are possible: either refuse to bind when the topology is not
  single-RP-passthrough, or extend the kernel ABI so a host-bridge
  HDM decoder programmer can attest the lock before vfio bind.  Both
  leave the existing contract intact or add a single boolean to
  CAP_CXL.

* Function-level reset (FLR) does not re-snapshot the shadows.
  Guests that issue FLR will see stale HDM and DVSEC state after
  the reset.

* Multi-decoder devices return ``-EOPNOTSUPP`` at bind.

* Hotplug while the device is held by vfio is not supported.

* Raw BAR read/write into the CXL component register sub-range is
  unsupported.  VMMs must use the COMP_REGS region.

Selftest
========

``tools/testing/selftests/vfio/vfio_cxl_type2_test`` exercises the
five surfaces:

* ``device_is_cxl`` — GET_INFO returns FLAGS_CXL + CAP_CXL.
* ``hdm_region_mmap_rw`` — mmap + read/write pattern.
* ``component_bar_sparse_mmap`` — SPARSE_MMAP cap excludes the CXL
  block.
* ``comp_regs_cm_cap_array_read`` — CM cap-array header is served
  from the cxl-core snapshot.
* ``dvsec_lock_byte_read`` -- DVSEC config-rw clipping shim is wired.
