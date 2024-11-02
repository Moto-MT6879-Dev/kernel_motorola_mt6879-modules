DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := nfg1000_fg_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_REQUIRED_MODULES := mmi_gauge_class.ko
include $(DLKM_DIR)/AndroidKernelModule.mk

#include $(call first-makefiles-under,$(LOCAL_PATH))
