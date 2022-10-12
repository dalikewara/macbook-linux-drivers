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
	{ 0xde0ab7a6, "snd_hda_jack_add_kctl_mst" },
	{ 0x57b41c5c, "kmalloc_caches" },
	{ 0xf9a482f9, "msleep" },
	{ 0x47884890, "system_power_efficient_wq" },
	{ 0xf5804708, "snd_hda_gen_free" },
	{ 0x7832f449, "snd_hda_codec_get_pin_target" },
	{ 0x809fe07d, "snd_hda_mixer_amp_volume_put" },
	{ 0x233bb28b, "snd_hdac_codec_read" },
	{ 0x5637f4ab, "snd_hda_get_default_vref" },
	{ 0x9dcd8b65, "snd_hda_gen_parse_auto_config" },
	{ 0xe1316405, "snd_hda_apply_fixup" },
	{ 0xe3c6f104, "snd_hda_add_verbs" },
	{ 0xbe7dd7dc, "snd_array_new" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0xb43f9365, "ktime_get" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xd12ec930, "snd_hda_mixer_amp_tlv" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x4c3d748a, "snd_hda_gen_build_pcms" },
	{ 0x1b96ab6b, "snd_hda_jack_unsol_event" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x2fb24c94, "snd_hda_mixer_amp_switch_put" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x2e054d24, "snd_hda_get_path_from_idx" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xcc0c8ec6, "snd_hda_check_amp_caps" },
	{ 0x3f01ec0b, "_dev_warn" },
	{ 0x502e1847, "snd_hda_mixer_amp_volume_get" },
	{ 0x2f6b0cab, "snd_hda_codec_set_name" },
	{ 0x3dad9978, "cancel_delayed_work" },
	{ 0xa3b842c7, "snd_hda_get_path_idx" },
	{ 0x7c9e27c, "_snd_hda_set_pin_ctl" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x67e2bcdb, "snd_hda_gen_fix_pin_power" },
	{ 0x80389e20, "snd_hda_codec_set_pin_target" },
	{ 0xff03b4ae, "snd_hda_mixer_amp_switch_get" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0x570e98d9, "snd_hda_get_conn_list" },
	{ 0xd37fd41c, "snd_hda_multi_out_analog_prepare" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x49619f5f, "snd_hda_activate_path" },
	{ 0xd72bbdd6, "snd_hda_gen_init" },
	{ 0x5b7afc7a, "snd_hda_parse_pin_defcfg" },
	{ 0xc694a0f1, "__hda_codec_driver_register" },
	{ 0xbdfbc184, "_dev_err" },
	{ 0x800473f, "__cond_resched" },
	{ 0x8c100acf, "snd_hda_sequence_write" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x9a9d589f, "_dev_info" },
	{ 0x93b1b4fb, "snd_hda_get_conn_index" },
	{ 0x4dfc80b5, "snd_hdac_override_parm" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0xadef80be, "_snd_hdac_read_parm" },
	{ 0x4052ac9f, "snd_hda_correct_pin_ctl" },
	{ 0x350fbb48, "snd_hda_add_imux_item" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xa926a2db, "kmem_cache_alloc_trace" },
	{ 0x4f883fe6, "__dynamic_dev_dbg" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0xaa660ca7, "snd_hda_gen_add_kctl" },
	{ 0x69acdf38, "memcpy" },
	{ 0x52c8c675, "snd_hda_jack_tbl_get_mst" },
	{ 0x3ae9685b, "snd_hda_jack_tbl_get_from_tag" },
	{ 0x96848186, "scnprintf" },
	{ 0x254446d, "snd_hdac_codec_write" },
	{ 0x6186a0fa, "snd_hda_add_new_path" },
	{ 0x8f4ee7d7, "hda_codec_driver_unregister" },
	{ 0x3fec1447, "snd_hda_mixer_amp_switch_info" },
	{ 0x4e7c53db, "snd_hda_codec_setup_stream" },
	{ 0x1c988227, "snd_hda_gen_spec_init" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x90c1b8ae, "snd_hda_mixer_amp_volume_info" },
	{ 0xc8b5e306, "snd_hda_jack_report_sync" },
	{ 0x2e2a505c, "hda_get_autocfg_input_label" },
	{ 0xe2254bcc, "snd_hda_codec_amp_init_stereo" },
	{ 0x390c94ff, "snd_hda_gen_build_controls" },
	{ 0xa2f7371b, "snd_hda_shutup_pins" },
};

MODULE_INFO(depends, "snd-hda-codec,snd-hda-codec-generic,snd-hda-core");

MODULE_ALIAS("hdaudio:v10138409r*a01*");

MODULE_INFO(srcversion, "7B93F590520E405AA32912D");
