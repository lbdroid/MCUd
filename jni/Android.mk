LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := mcud
LOCAL_LDFLAGS := -static
LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_SRC_FILES := mcud.c
LOCAL_CFLAGS := -std=gnu11
LOCAL_MODULE_PATH := /system/bin/
include $(BUILD_EXECUTABLE)

