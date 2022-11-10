def append_int8(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 8)


def get_int8(buffer: bytearray, idx: int) -> tuple[int, int]:
    return get_int(buffer, idx, 8)


def append_int16(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 16)


def get_int16(buffer: bytearray, idx: int) -> tuple[int, int]:
    return get_int(buffer, idx, 16)


def append_int32(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 32)


def get_int32(buffer: bytearray, idx: int) -> tuple[int, int]:
    return get_int(buffer, idx, 32)


def append_int64(buffer, value, idx) -> int:
    return append_int(buffer, value, idx, 64)


def get_int64(buffer: bytearray, idx: int) -> tuple[int, int]:
    return get_int(buffer, idx, 64)


def append_float16(buffer: bytearray, value: float, scale: float, idx: int) -> int:
    return append_int16(buffer, int(value * scale), idx)


def get_float16(buffer: bytearray, scale: float, idx: int) -> tuple[float, int]:
    idx, result = get_int16(buffer, idx)
    return idx, float(result / scale)


def append_float32(buffer: bytearray, value: float, scale: float, idx: int) -> int:
    return append_int32(buffer, int(value * scale), idx)


def get_float32(buffer: bytearray, scale: float, idx: int) -> tuple[float, int]:
    idx, result = get_int32(buffer, idx)
    return idx, float(result / scale)

def append_float64(buffer: bytearray, value: float, scale: float, idx: int) -> int:
    return append_int64(buffer, int(value * scale), idx)


def get_float64(buffer: bytearray, scale: float, idx: int) -> tuple[float, int]:
    idx, result = get_int64(buffer, idx)
    return idx, float(result / scale)


def append_int(buffer: bytearray, value: int, idx: int, int_size: int) -> int:
    n_shifts: int = int_size // 8 - 1
    for i in range(n_shifts + 1):
        n_bits = (n_shifts - i) * 8
        buffer[idx] = (value >> n_bits) & 0xff
        idx += 1
    return idx


def get_int(buffer: bytearray, idx: int, int_size: int) -> tuple[int, int]:
    n_shifts: int = int_size // 8
    result: int = 0
    for _ in range(idx, idx + n_shifts):
        result = (result << 8) | buffer[idx]
        idx += 1
    return idx, result


def append_buffer(buffer: bytearray, data: bytearray, idx: int) -> int:
    assert len(buffer) - idx >= len(data), 'not enough memory in buffer for data'
    for byte in data:
        buffer[idx] = byte
        idx += 1
    return idx
