DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DLKM_INSTALL_TO_VENDOR_OUT),true)
NOVA_MMI_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/modules/
else
NOVA_MMI_MODULE_PATH := $(KERNEL_MODULES_OUT)
endif

ifneq ($(BOARD_USES_DOUBLE_TAP),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_ENABLE_DOUBLE_TAP=y
endif

ifneq ($(MOTO_PANEL_CHECK_TOUCH_STATE),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_NOTIFY_TOUCH_STATE=y
endif

ifneq ($(BOARD_USES_PANEL_NOTIFICATIONS),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_PANEL_NOTIFICATIONS=y
endif

ifeq ($(CONFIG_INPUT_ENABLE_DRM_PANEL),true)
        KERNEL_CFLAGS += CONFIG_INPUT_NOVA_DRM_PANEL=y
endif

ifeq ($(CONFIG_INPUT_ENABLE_MULTI_SUPPLIER),true)
        KERNEL_CFLAGS += CONFIG_INPUT_NOVA_MULTI_SUPPLIER=y
endif

ifeq ($(CONFIG_INPUT_NOVA_USES_CHIP_VER_1),true)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_CHIP_VER_1=y
endif

ifeq ($(CONFIG_INPUT_NOVA_ESD_ENABLE),true)
	KERNEL_CFLAGS += CONFIG_NOVA_ESD_ENABLE=y
endif

ifeq ($(CONFIG_INPUT_NOVA_LCM_FAST_LIGHTUP),true)
	KERNEL_CFLAGS += CONFIG_NOVA_LCM_FAST_LIGHTUP=y
endif

ifneq ($(BOARD_USES_PEN_NOTIFIER),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_PEN_NOTIFIER=y
endif

ifneq ($(BOARD_USES_STYLUS_PALM),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_STYLUS_PALM=y
endif

ifneq ($(BOARD_USES_STYLUS_PALM_RANGE),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_STYLUS_PALM_RANGE=y
endif
ifneq ($(BOARD_USES_MTK_GET_PANEL),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_MTK_GET_PANEL=y
endif
ifneq ($(CONFIG_INPUT_CHECK_PANEL),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_MTK_GET_PANEL=y
endif
ifneq ($(CONFIG_INPUT_FW_WITH_IC_INFO),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_FW_WITH_IC_INFO=y
endif
ifneq ($(CONFIG_NOVA_CHARGER_FLAG_BATTERY),)
        KERNEL_CFLAGS += CONFIG_INPUT_NOVA_CHARGER_FLAG_BATTERY=y
endif

ifneq ($(BOARD_USES_MT6768_SPI_CONF),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_MT6768_SPI_CONF=y
endif

ifeq ($(TOUCHSCREEN_STYLUS_TIME),true)
        KERNEL_CFLAGS += CONFIG_NOVA_TOUCH_STYLUS_TIME=y
endif

ifeq ($(TOUCHSCREEN_DDA_STYLUS_TIME),true)
        KERNEL_CFLAGS += CONFIG_DDA_STYLUS_TIME=y
endif

ifeq ($(TOUCHSCREEN_DDA_REPORT_LOG_CHECK),true)
        KERNEL_CFLAGS += CONFIG_TP_DDA_LOG_CHECK=y
endif

ifeq ($(GTP_ENABLE_DDA_STYLUS),true)
	KERNEL_CFLAGS += CONFIG_GTP_DDA_STYLUS=y
endif

include $(CLEAR_VARS)
ifneq ($(BOARD_USES_DOUBLE_TAP),)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/sensors_class.ko
LOCAL_REQUIRED_MODULES := sensors_class.ko
endif
ifneq ($(BOARD_USES_PEN_NOTIFIER),)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/bu520xx_pen.ko
LOCAL_REQUIRED_MODULES := bu520xx_pen.ko
endif

ifneq ($(BOARD_USES_NOVA_PENDETECT_CONF),)
	KERNEL_CFLAGS += CONFIG_INPUT_NOVA_0FLASH_MMI_PEN_NOTIFIER=y
endif
LOCAL_MODULE := nova_0flash_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(NOVA_MMI_MODULE_PATH)
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
