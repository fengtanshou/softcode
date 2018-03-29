LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := CameraCapture.cpp

LOCAL_MODULE := CameraCapture

LOCAL_STATIC_LIBRARIES := libc
LOCAL_MODULE_PATH:= $(TARGET_ROOT_OUT_SBIN)/pretest
LOCAL_FORCE_STATIC_EXECUTABLE := true

LOCAL_SHARED_LIBRARIES:= \
	liblog \
	libcutils

include $(BUILD_EXECUTABLE)
