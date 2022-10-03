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
	{ 0xe49bb82b, "module_layout" },
	{ 0xde0ab7a6, "snd_hda_jack_add_kctl_mst" },
	{ 0x3703b5ff, "kmalloc_caches" },
	{ 0xf9a482f9, "msleep" },
	{ 0x47884890, "system_power_efficient_wq" },
	{ 0x6d698f51, "snd_hda_gen_free" },
	{ 0x78309f7c, "snd_hda_codec_get_pin_target" },
	{ 0x64598589, "snd_hda_mixer_amp_volume_put" },
	{ 0xcb321b01, "snd_hdac_codec_read" },
	{ 0x56359f9e, "snd_hda_get_default_vref" },
	{ 0xa859fac1, "snd_hda_gen_parse_auto_config" },
	{ 0xe1316405, "snd_hda_apply_fixup" },
	{ 0xe3c6f104, "snd_hda_add_verbs" },
	{ 0xbe7dd7dc, "snd_array_new" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0xb43f9365, "ktime_get" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x69642c74, "snd_hda_mixer_amp_tlv" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xd4d4bcd3, "snd_hda_gen_build_pcms" },
	{ 0x1b96ab6b, "snd_hda_jack_unsol_event" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xcb742960, "snd_hda_mixer_amp_switch_put" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x67000b93, "snd_hda_get_path_from_idx" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xf4c15343, "snd_hda_check_amp_caps" },
	{ 0x5595708b, "_dev_warn" },
	{ 0xb4e87db3, "snd_hda_mixer_amp_volume_get" },
	{ 0xe21ad3a3, "snd_hda_codec_set_name" },
	{ 0x3dad9978, "cancel_delayed_work" },
	{ 0xb4b5a677, "snd_hda_get_path_idx" },
	{ 0x9a23580e, "_snd_hda_set_pin_ctl" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x67e0d7ee, "snd_hda_gen_fix_pin_power" },
	{ 0xdca180f5, "snd_hda_codec_set_pin_target" },
	{ 0x1bc5d15a, "snd_hda_mixer_amp_switch_get" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0xcae422ab, "snd_hda_get_conn_list" },
	{ 0x5c821112, "snd_hda_multi_out_analog_prepare" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x16867170, "snd_hda_activate_path" },
	{ 0x4fc2758f, "snd_hda_gen_init" },
	{ 0x5b7afc7a, "snd_hda_parse_pin_defcfg" },
	{ 0x7dae0eb, "__hda_codec_driver_register" },
	{ 0x63b28712, "_dev_err" },
	{ 0x800473f, "__cond_resched" },
	{ 0xa664db74, "snd_hda_sequence_write" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xca3d15a5, "_dev_info" },
	{ 0x45615743, "snd_hda_get_conn_index" },
	{ 0xaa025d37, "snd_hdac_override_parm" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x7637437a, "_snd_hdac_read_parm" },
	{ 0x1ccbb24a, "snd_hda_correct_pin_ctl" },
	{ 0x24528945, "snd_hda_add_imux_item" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x60f64c0d, "kmem_cache_alloc_trace" },
	{ 0x37669c31, "__dynamic_dev_dbg" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x4a8350e2, "snd_hda_gen_add_kctl" },
	{ 0x69acdf38, "memcpy" },
	{ 0x52c8c675, "snd_hda_jack_tbl_get_mst" },
	{ 0x3ae9685b, "snd_hda_jack_tbl_get_from_tag" },
	{ 0x96848186, "scnprintf" },
	{ 0xea5dede7, "snd_hdac_codec_write" },
	{ 0x71dab833, "snd_hda_add_new_path" },
	{ 0x4da205ac, "hda_codec_driver_unregister" },
	{ 0xf746ebcb, "snd_hda_mixer_amp_switch_info" },
	{ 0x98acb063, "snd_hda_codec_setup_stream" },
	{ 0x4b7d7c09, "snd_hda_gen_spec_init" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x586b4722, "snd_hda_mixer_amp_volume_info" },
	{ 0xc8b5e306, "snd_hda_jack_report_sync" },
	{ 0x2e2a505c, "hda_get_autocfg_input_label" },
	{ 0xd49897c0, "snd_hda_codec_amp_init_stereo" },
	{ 0xa1e55ca6, "snd_hda_gen_build_controls" },
	{ 0x3a1eff42, "snd_hda_shutup_pins" },
};

MODULE_INFO(depends, "snd-hda-codec,snd-hda-codec-generic,snd-hda-core");

MODULE_ALIAS("hdaudio:v10138409r*a01*");

MODULE_INFO(srcversion, "7B93F590520E405AA32912D");
