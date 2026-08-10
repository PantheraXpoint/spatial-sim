#!/usr/bin/env python3
"""Open a real NVENC session on every GPU the container can see.

Run inside the sim container:  make encoder-check

WHY THIS EXISTS. `ldconfig -p | grep libnvidia-encode` -- the check this
replaced -- passes whenever the library was mounted, which is always. It
reported a healthy encoder through a total livestream outage: the library is
present, loads fine, and `NvEncodeAPICreateInstance` succeeds; it is
`nvEncOpenEncodeSessionEx` that rejects the device. Nothing short of opening a
session distinguishes the two, and the failure is silent everywhere else --
signaling connects, the app runs, no port is missing, and the client shows
black forever.

THE FAILURE THIS CATCHES. NVENC needs /dev/nvidia0 -- the node whose device
MINOR is 0 -- inside the container. Expose any other subset of GPUs and every
one of them returns NV_ENC_ERR_UNSUPPORTED_DEVICE, including cards that encode
perfectly on the host. Driver bug, 570.x/580.x:
github.com/NVIDIA/nvidia-container-toolkit/issues/1249

nvidia-smi index is NOT the device minor. Map them with:
    grep -H 'Device Minor' /proc/driver/nvidia/gpus/*/information
and pick device_ids in docker-compose.yml so that minor 0 is among them.

Stdlib only, on purpose: this has to run before anything else is trusted.
"""

import ctypes as C
import sys

# Values from nvEncodeAPI.h. Only the ones a failing probe actually returns.
NVENC_STATUS = {
    0: "SUCCESS",
    1: "NO_ENCODE_DEVICE",
    2: "UNSUPPORTED_DEVICE",
    3: "INVALID_ENCODERDEVICE",
    4: "INVALID_DEVICE",
    5: "DEVICE_NOT_EXIST",
    6: "INVALID_PTR",
    8: "INVALID_PARAM",
    9: "INVALID_VERSION",
    11: "OUT_OF_MEMORY",
    13: "UNSUPPORTED_PARAM",
    22: "GENERIC",
}

NV_ENC_DEVICE_TYPE_CUDA = 1

# NVENCAPI_VERSION        = major | (minor << 24)
# NVENCAPI_STRUCT_VERSION = NVENCAPI_VERSION | (struct << 16) | (0x7 << 28)
def _api_version(major, minor):
    return major | (minor << 24)


def _struct_version(major, minor, struct):
    return _api_version(major, minor) | (struct << 16) | (0x7 << 28)


class _FunctionList(C.Structure):
    # version, reserved, then the function-pointer table. Index 29 is
    # nvEncOpenEncodeSessionEx -- the only entry we call.
    _fields_ = [
        ("version", C.c_uint32),
        ("reserved", C.c_uint32),
        ("fn", C.c_void_p * 62),
        ("reserved2", C.c_void_p * 64),
    ]


class _OpenSessionExParams(C.Structure):
    _fields_ = [
        ("version", C.c_uint32),
        ("deviceType", C.c_uint32),
        ("device", C.c_void_p),
        ("reserved", C.c_void_p),
        ("apiVersion", C.c_uint32),
        ("reserved1", C.c_uint32 * 253),
        ("reserved2", C.c_void_p * 64),
    ]


def main():
    try:
        cuda = C.CDLL("libcuda.so.1")
        nvenc = C.CDLL("libnvidia-encode.so.1")
    except OSError as exc:
        print(f"FAIL: cannot load NVIDIA libraries: {exc}")
        print("      The container has no driver libraries mounted at all.")
        return 1

    if cuda.cuInit(0) != 0:
        print("FAIL: cuInit failed -- no usable CUDA device in this container.")
        return 1

    count = C.c_int()
    cuda.cuDeviceGetCount(C.byref(count))
    if count.value == 0:
        print("FAIL: container sees zero CUDA devices. Check device_ids.")
        return 1

    version = C.c_uint32()
    nvenc.NvEncodeAPIGetMaxSupportedVersion(C.byref(version))
    major, minor = version.value >> 4, version.value & 0xF

    fn_list = _FunctionList()
    fn_list.version = _struct_version(major, minor, 2)
    rc = nvenc.NvEncodeAPICreateInstance(C.byref(fn_list))
    if rc != 0:
        print(f"FAIL: NvEncodeAPICreateInstance -> {NVENC_STATUS.get(rc, rc)}")
        return 1

    open_session = C.CFUNCTYPE(C.c_int, C.c_void_p, C.POINTER(C.c_void_p))(
        fn_list.fn[29]
    )

    print(f"NVENCAPI {major}.{minor}, {count.value} CUDA device(s) visible")
    failures = 0
    for i in range(count.value):
        device = C.c_int()
        cuda.cuDeviceGet(C.byref(device), i)
        name = C.create_string_buffer(128)
        cuda.cuDeviceGetName(name, 128, device)

        context = C.c_void_p()
        if cuda.cuCtxCreate_v2(C.byref(context), 0, device) != 0:
            print(f"  device {i} {name.value.decode()}: cuCtxCreate FAILED")
            failures += 1
            continue

        params = _OpenSessionExParams()
        params.version = _struct_version(major, minor, 1)
        params.deviceType = NV_ENC_DEVICE_TYPE_CUDA
        params.device = context
        params.apiVersion = _api_version(major, minor)

        encoder = C.c_void_p()
        rc = open_session(C.byref(params), C.byref(encoder))
        status = NVENC_STATUS.get(rc, str(rc))
        print(f"  device {i} {name.value.decode():28s} session -> {status}")
        if rc != 0:
            failures += 1
        cuda.cuCtxDestroy_v2(context)

    if failures:
        print()
        print(f"FAIL: {failures}/{count.value} GPU(s) cannot open an NVENC session.")
        print("      Livestreaming will connect and show a permanently black")
        print("      screen. If the status is UNSUPPORTED_DEVICE, /dev/nvidia0")
        print("      is almost certainly missing from the container:")
        print("        ls /dev/nvidia[0-9]*")
        print("      Fix device_ids in docker/docker-compose.yml so the GPU with")
        print("      device minor 0 is included. See the comment there.")
        return 1

    print()
    print("OK: every visible GPU can open an NVENC session.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
