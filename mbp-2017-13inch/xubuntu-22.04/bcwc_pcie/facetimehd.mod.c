#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc4162456, "module_layout" },
	{ 0x995f149, "vb2_ioctl_reqbufs" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x57b41c5c, "kmalloc_caches" },
	{ 0xec38e348, "v4l2_event_unsubscribe" },
	{ 0x6fb3fec8, "pci_write_config_dword" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xdf44b8, "pci_release_region" },
	{ 0xdc5f02c5, "debugfs_create_dir" },
	{ 0xc5e74216, "release_resource" },
	{ 0x8e72f861, "video_device_release" },
	{ 0x961a8b82, "dma_set_mask" },
	{ 0x3bff1714, "vb2_wait_for_all_buffers" },
	{ 0x304a3845, "pci_disable_device" },
	{ 0xd2b987bb, "v4l2_device_unregister" },
	{ 0xf4bef96, "v4l2_ctrl_handler_free" },
	{ 0xea88947f, "v4l2_ctrl_new_std" },
	{ 0xbf004aa4, "vb2_fop_poll" },
	{ 0x96e14390, "vb2_ioctl_streamon" },
	{ 0xa1e158d0, "seq_printf" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x837b7b09, "__dynamic_pr_debug" },
	{ 0xd92deb6b, "acpi_evaluate_object" },
	{ 0x2ce42f7c, "vb2_ops_wait_prepare" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x6e386bac, "__video_register_device" },
	{ 0xe5471ee2, "debugfs_create_file" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xf3e0e1df, "allocate_resource" },
	{ 0x386a1fc1, "pv_ops" },
	{ 0x2d39b0a7, "kstrdup" },
	{ 0x1584f964, "dma_set_coherent_mask" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x3e1931af, "v4l2_device_register" },
	{ 0xad952f61, "vb2_fop_read" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xc296c668, "pci_set_master" },
	{ 0x3f01ec0b, "_dev_warn" },
	{ 0xa0d772bc, "default_llseek" },
	{ 0x17d993cd, "video_device_alloc" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x8736f72b, "vb2_fop_mmap" },
	{ 0xa73dd223, "vb2_ioctl_qbuf" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0x89a894bc, "video_unregister_device" },
	{ 0xde80cd09, "ioremap" },
	{ 0x1a9a433c, "prandom_u32_state" },
	{ 0x4464470, "v4l2_ctrl_subscribe_event" },
	{ 0x643477f0, "vb2_buffer_done" },
	{ 0xaafdc258, "strcasecmp" },
	{ 0x9166fada, "strncpy" },
	{ 0x5a921311, "strncmp" },
	{ 0x81682779, "debugfs_remove" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x4741a548, "vb2_ioctl_create_bufs" },
	{ 0xaba436d1, "simple_open" },
	{ 0xbdfbc184, "_dev_err" },
	{ 0x50b59146, "pci_enable_msi" },
	{ 0x14afc34b, "vb2_ioctl_dqbuf" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0x4ad53e65, "vb2_plane_cookie" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0x9a9d589f, "_dev_info" },
	{ 0x271eefa2, "pci_disable_link_state" },
	{ 0xf1865426, "vb2_fop_release" },
	{ 0x174ed0bb, "video_devdata" },
	{ 0x29908925, "debugfs_create_devm_seqfile" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x92997ed8, "_printk" },
	{ 0xb9e7429c, "memcpy_toio" },
	{ 0x7057818f, "pci_read_config_dword" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x25255b41, "pci_unregister_driver" },
	{ 0xa926a2db, "kmem_cache_alloc_trace" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x83dd63e0, "v4l2_fh_open" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x6af20ee5, "pci_set_power_state" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x93593ce, "vb2_ioctl_querybuf" },
	{ 0x37a0cba, "kfree" },
	{ 0xc3b0870d, "pci_disable_msi" },
	{ 0xd58e70dd, "net_rand_noise" },
	{ 0xedc03953, "iounmap" },
	{ 0x7d628444, "memcpy_fromio" },
	{ 0x5b36e1ee, "v4l2_ctrl_handler_init_class" },
	{ 0x43387c54, "__pci_register_driver" },
	{ 0x10decfd6, "request_firmware" },
	{ 0x1490644b, "vb2_ops_wait_finish" },
	{ 0x92540fbf, "finish_wait" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x1ee1aa5, "vb2_ioctl_expbuf" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0x384eb8ff, "vb2_ioctl_streamoff" },
	{ 0x4a453f53, "iowrite32" },
	{ 0x86e020fb, "pci_enable_device" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x342ad3de, "is_acpi_device_node" },
	{ 0xc6d09aa9, "release_firmware" },
	{ 0x6769086c, "video_ioctl2" },
	{ 0x36e4be61, "pci_request_region" },
	{ 0x9e7d6bd0, "__udelay" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xa78af5f3, "ioread32" },
	{ 0x1efec19, "vb2_dma_sg_memops" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x81e6b37f, "dmi_get_system_info" },
	{ 0x476f8154, "vb2_queue_init" },
};

MODULE_INFO(depends, "videobuf2-v4l2,videodev,videobuf2-common,videobuf2-dma-sg");

MODULE_ALIAS("pci:v000014E4d00001570sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "C7D4F5C792E1E2CB51292C1");
