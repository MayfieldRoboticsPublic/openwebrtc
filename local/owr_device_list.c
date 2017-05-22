/*
 * Copyright (c) 2014-2015, Ericsson AB. All rights reserved.
 * Copyright (c) 2014, Centricular Ltd
 *     Author: Sebastian Dröge <sebastian@centricular.com>
 *     Author: Arun Raghavan <arun@centricular.com>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

/*/
\*\ OwrDeviceList
/*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "owr_device_list_private.h"
#include "owr_local_media_source_private.h"
#include "owr_media_source.h"
#include "owr_private.h"
#include "owr_types.h"
#include "owr_utils.h"

#include <gst/gstinfo.h>

GST_DEBUG_CATEGORY_EXTERN(_owrdevicelist_debug);
#define GST_CAT_DEFAULT _owrdevicelist_debug

#ifdef __APPLE__
#include "owr_device_list_avf_private.h"
#include <TargetConditionals.h>

#elif defined(__ANDROID__)
#include <assert.h>
#include <dlfcn.h>
#include <jni.h>
#include <stdlib.h>

#elif defined(__linux__)
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#endif /* defined(__linux__) */

static gboolean enumerate_video_source_devices(GClosure *);
static gboolean enumerate_audio_source_devices(GClosure *);

typedef struct {
    GClosure *callback;
    GList *list;
} CallbackAndList;

void _owr_get_capture_devices(OwrMediaType types, GClosure *callback)
{
    GClosure *merger;

    g_return_if_fail(callback);

    if (G_CLOSURE_NEEDS_MARSHAL(callback))
        g_closure_set_marshal(callback, g_cclosure_marshal_generic);

    merger = _owr_utils_list_closure_merger_new(callback,
        (GCopyFunc) g_object_ref,
        (GDestroyNotify) g_object_unref);

    if (types & OWR_MEDIA_TYPE_VIDEO) {
        g_closure_ref(merger);
        _owr_schedule_with_user_data((GSourceFunc) enumerate_video_source_devices, merger);
    }

    if (types & OWR_MEDIA_TYPE_AUDIO) {
        g_closure_ref(merger);
        _owr_schedule_with_user_data((GSourceFunc) enumerate_audio_source_devices, merger);
    }

    g_closure_unref(merger);
}



#if defined(__APPLE__)

static gboolean enumerate_video_source_devices(GClosure *callback)
{
    GList *sources = _owr_get_avf_video_sources();
    _owr_utils_call_closure_with_list(callback, sources);
    g_list_free_full(sources, g_object_unref);
    return FALSE;
}

static gboolean enumerate_audio_source_devices(GClosure *callback)
{
    GList *sources = _owr_get_core_audio_sources();
    _owr_utils_call_closure_with_list(callback, sources);
    g_list_free_full(sources, g_object_unref);
    return FALSE;
}

#endif /*defined(__APPLE__)*/



#if defined(__ANDROID__)

static gboolean enumerate_audio_source_devices(GClosure *callback)
{
    OwrLocalMediaSource *source;
    GList *sources = NULL;

    source = _owr_local_media_source_new_cached(-1,
        "Default audio input", OWR_MEDIA_TYPE_AUDIO, OWR_SOURCE_TYPE_CAPTURE,
        NULL);
    sources = g_list_prepend(sources, source);
    _owr_utils_call_closure_with_list(callback, sources);
    g_list_free_full(sources, g_object_unref);

    return FALSE;
}

#endif /*defined(__ANDROID__)*/



#if (defined(__linux__) && !defined(__ANDROID__))

static gboolean enumerate_source_devices(OwrMediaType type, GClosure *callback)
{
    OwrLocalMediaSource *source;
    GList *sources = NULL;
    GstDeviceMonitor *monitor;
    GstCaps *caps;
    GList *list, *l;
    const gchar *klass;

    if (type == OWR_MEDIA_TYPE_AUDIO)
        klass = "Source/Audio";
    else if (type == OWR_MEDIA_TYPE_VIDEO)
        klass = "Source/Video";
    else
        g_assert_not_reached();

    monitor = gst_device_monitor_new();
    caps = gst_caps_new_any();

    gst_device_monitor_add_filter(monitor, klass, caps);
    gst_caps_unref(caps);

    list = gst_device_monitor_get_devices(monitor);

    for (l = list; l; l = l->next) {
        GstDevice *device = GST_DEVICE(l->data);

        gchar *name = gst_device_get_display_name(device);

        source = _owr_local_media_source_new_cached(-1,
                name, type,
                OWR_SOURCE_TYPE_CAPTURE,
                device);
        g_free(name);
        sources = g_list_prepend(sources, source);
    }

    g_list_free(list);
    g_object_unref(monitor);

    _owr_utils_call_closure_with_list(callback, sources);
    g_list_free_full(sources, g_object_unref);

    return FALSE;
}

static gboolean enumerate_audio_source_devices(GClosure *callback)
{
    return enumerate_source_devices(OWR_MEDIA_TYPE_AUDIO, callback);
}

static gboolean enumerate_video_source_devices(GClosure *callback)
{
    return enumerate_source_devices(OWR_MEDIA_TYPE_VIDEO, callback);
}

#endif /*(defined(__linux__) && !defined(__ANDROID__))*/



#if defined(__ANDROID__)

#define OWR_DEVICE_LIST_JNI_VERSION JNI_VERSION_1_6
#define OWR_DEVICE_LIST_MIN_SDK_VERSION 9

#define ANDROID_RUNTIME_DALVIK_LIB "libdvm.so"
#define ANDROID_RUNTIME_ART_LIB "libart.so"

typedef jint (*JNI_GetCreatedJavaVMs)(JavaVM **vmBuf, jsize bufLen, jsize *nVMs);

static void cache_java_classes(JNIEnv *);

static const char *const android_runtime_libs[] = {
    ANDROID_RUNTIME_DALVIK_LIB,
    ANDROID_RUNTIME_ART_LIB,
    NULL
};

static pthread_key_t detach_key = 0;

static void on_java_detach(JavaVM *jvm)
{
    g_return_if_fail(jvm);

    g_debug("%s detached thread(%ld) from Java VM", __FUNCTION__, pthread_self());
    (*jvm)->DetachCurrentThread(jvm);
    pthread_setspecific(detach_key, NULL);
}

static int get_android_sdk_version(JNIEnv *env)
{
    jfieldID field_id = NULL;
    jint version;
    static jclass ref;

    assert(env);

    if (g_once_init_enter(&ref)) {
        jclass c = (*env)->FindClass(env, "android/os/Build$VERSION");
        g_assert(c);

        field_id = (*env)->GetStaticFieldID(env, c, "SDK_INT", "I");
        g_assert(field_id);

        g_once_init_leave(&ref, c);
    }

    version = (*env)->GetStaticIntField(env, ref, field_id);
    g_debug("android device list: android sdk version is: %d", version);

    (*env)->DeleteLocalRef(env, ref);

    return version;
}

static JNIEnv* get_jni_env_from_jvm(JavaVM *jvm)
{
    JNIEnv *env = NULL;
    int err;

    g_return_val_if_fail(jvm, NULL);

    err = (*jvm)->GetEnv(jvm, (void**)&env, OWR_DEVICE_LIST_JNI_VERSION);

    if (JNI_EDETACHED == err) {
        err = (*jvm)->AttachCurrentThread(jvm, &env, NULL);

        if (err) {
            g_warning("android device list: failed to attach current thread");
            return NULL;
        }

        g_debug("attached thread (%ld) to jvm", pthread_self());

        if (pthread_key_create(&detach_key, (void (*)(void *)) on_java_detach))
            g_warning("android device list: failed to set on_java_detach");

        pthread_setspecific(detach_key, jvm);
    } else if (JNI_OK != err)
        g_warning("jvm->GetEnv() failed");

    return env;
}

static void init_jni(JavaVM *jvm)
{
    JNIEnv *env;
    int sdk_version;

    env = get_jni_env_from_jvm(jvm);

    sdk_version = get_android_sdk_version(env);

    if (sdk_version < OWR_DEVICE_LIST_MIN_SDK_VERSION) {
        g_warning("android version is %d, owr_device_list needs > %d",
            sdk_version, OWR_DEVICE_LIST_MIN_SDK_VERSION);
    }

    cache_java_classes(env);
}

static JavaVM *get_java_vm(void)
{
    JNI_GetCreatedJavaVMs get_created_java_vms;
    gpointer handle = NULL;
    static JavaVM *jvm = NULL;
    const gchar *error_string;
    jsize num_jvms = 0;
    gint lib_index = 0;
    gint err;

    while (android_runtime_libs[lib_index] && !handle) {
        dlerror();
        handle = dlopen(android_runtime_libs[lib_index], RTLD_LOCAL | RTLD_LAZY);
        error_string = dlerror();

        if (error_string)
            g_debug("failed to load %s: %s", android_runtime_libs[lib_index], error_string);

        if (handle)
            g_debug("Android runtime loaded from %s", android_runtime_libs[lib_index]);
        else
            ++lib_index;
    }

    if (handle) {
        dlerror();
        *(void **) (&get_created_java_vms) = dlsym(handle, "JNI_GetCreatedJavaVMs");
        error_string = dlerror();

        if (error_string) {
            g_warning("dlsym(\"JNI_GetCreatedJavaVMs\") failed: %s", error_string);
            return NULL;
        }

        get_created_java_vms(&jvm, 1, &num_jvms);

        if (num_jvms < 1)
            g_debug("get_created_java_vms returned %d jvms", num_jvms);
        else
            g_debug("found existing jvm");

        err = dlclose(handle);
        if (err)
            g_warning("dlclose() of android runtime handle failed");
    } else
        g_error("Failed to get jvm");

    return jvm;
}

/* jvm needs to be fetched once, jni env needs to be fetched once per thread */
static JNIEnv* get_jni_env(void)
{
    static JavaVM *jvm = NULL;

    if (g_once_init_enter(&jvm)) {
        JavaVM *vm;
        vm = get_java_vm();
        init_jni(vm);
        g_once_init_leave(&jvm, vm);
    }

    g_return_val_if_fail(jvm, NULL);

    return get_jni_env_from_jvm(jvm);
}

static struct {
    jobject class;
    jmethodID getNumberOfCameras;
    jmethodID getCameraInfo;
} Camera;

static void cache_class_camera(JNIEnv *env)
{
    jclass ref = NULL;

    ref = (*env)->FindClass(env, "android/hardware/Camera");
    g_assert(ref);

    Camera.class = (*env)->NewGlobalRef(env, ref);
    (*env)->DeleteLocalRef(env, ref);
    g_assert(Camera.class);

    Camera.getNumberOfCameras = (*env)->GetStaticMethodID(env, Camera.class, "getNumberOfCameras", "()I");
    g_assert(Camera.getNumberOfCameras);

    Camera.getCameraInfo = (*env)->GetStaticMethodID(env, Camera.class, "getCameraInfo", "(ILandroid/hardware/Camera$CameraInfo;)V");
    g_assert(Camera.getCameraInfo);
}

static struct {
    jobject class;
    jmethodID constructor;
    jfieldID facing;
    jint CAMERA_FACING_BACK;
    jint CAMERA_FACING_FRONT;
} CameraInfo;

static void cache_class_camera_info(JNIEnv *env)
{
    jclass ref = NULL;

    jfieldID field_id;

    ref = (*env)->FindClass(env, "android/hardware/Camera$CameraInfo");
    g_assert(ref);

    CameraInfo.class = (*env)->NewGlobalRef(env, ref);
    (*env)->DeleteLocalRef(env, ref);
    g_assert(CameraInfo.class);

    CameraInfo.constructor = (*env)->GetMethodID(env, CameraInfo.class, "<init>", "()V");

    CameraInfo.facing = (*env)->GetFieldID(env, CameraInfo.class, "facing", "I");

    field_id = (*env)->GetStaticFieldID(env, CameraInfo.class, "CAMERA_FACING_BACK", "I");
    CameraInfo.CAMERA_FACING_BACK = (*env)->GetStaticIntField(env, CameraInfo.class, field_id);
    g_assert(field_id);

    field_id = (*env)->GetStaticFieldID(env, CameraInfo.class, "CAMERA_FACING_FRONT", "I");
    CameraInfo.CAMERA_FACING_FRONT = (*env)->GetStaticIntField(env, CameraInfo.class, field_id);
    g_assert(field_id);
}

static void cache_java_classes(JNIEnv *env)
{
    cache_class_camera(env);
    cache_class_camera_info(env);
}

static gint get_number_of_cameras(void)
{
    JNIEnv *env = get_jni_env();
    return (*env)->CallStaticIntMethod(env, Camera.class, Camera.getNumberOfCameras);
}

static jint get_camera_facing(gint camera_index)
{
    jint facing;
    jobject camera_info_instance;
    JNIEnv *env;

    env = get_jni_env();
    g_return_val_if_fail(env, 0);

    camera_info_instance = (*env)->NewObject(env, CameraInfo.class, CameraInfo.constructor);
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
        g_warning("android device list: failed to create CameraInfo object");
        return -1;
    }

    (*env)->CallStaticVoidMethod(env, Camera.class, Camera.getCameraInfo, camera_index, camera_info_instance);
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
        g_warning("android device list: could not get camera info");
        return -1;
    }

    facing = (*env)->GetIntField(env, camera_info_instance, CameraInfo.facing);
    (*env)->DeleteLocalRef(env, camera_info_instance);

    return facing;
}

static gboolean enumerate_video_source_devices(GClosure *callback)
{
    gint num;
    gint i;
    GList *sources = NULL;
    OwrLocalMediaSource *source;

    num = get_number_of_cameras();

    for (i = 0; i < num; ++i) {
        jint facing = get_camera_facing(i);

        if (facing == CameraInfo.CAMERA_FACING_FRONT) {
            source = _owr_local_media_source_new_cached(i, "Front facing Camera",
                OWR_MEDIA_TYPE_VIDEO, OWR_SOURCE_TYPE_CAPTURE, NULL);
            sources = g_list_prepend(sources, source);
        } else if (facing == CameraInfo.CAMERA_FACING_BACK) {
            source = _owr_local_media_source_new_cached(i, "Back facing Camera",
                OWR_MEDIA_TYPE_VIDEO, OWR_SOURCE_TYPE_CAPTURE, NULL);
            sources = g_list_append(sources, source);
        }

    }

    _owr_utils_call_closure_with_list(callback, sources);
    g_list_free_full(sources, g_object_unref);

    return FALSE;
}

#endif /*defined(__ANDROID__)*/
