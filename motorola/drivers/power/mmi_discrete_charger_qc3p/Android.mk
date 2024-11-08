DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := mmi_discrete_charger_qc3p.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif
LOCAL_REQUIRED_MODULES := bq2597x_mmi_discrete.ko
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/bq2597x_mmi_discrete.ko

KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
