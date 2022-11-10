def append_int8(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 8)


def append_int16(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 16)


def append_int32(buffer: bytearray, value: int, idx: int) -> int:
    return append_int(buffer, value, idx, 32)


def append_int64(buffer, value, idx) -> int:
    return append_int(buffer, value, idx, 64)


def append_int(buffer: bytearray, value: int, idx: int, int_size: int) -> int:
    n_shifts = int_size // 8 - 1

    for i in range(n_shifts + 1):
        n_bits = (n_shifts - i) * 8
        buffer[idx] = (value >> n_bits) & 0xff
        idx += 1

    return idx


def append_buffer(buffer: bytearray, data: bytearray, idx: int) -> int:
    assert len(buffer) - idx >= len(data), 'not enough memory in buffer for data'

    for byte in data:
        buffer[idx] = byte
        idx += 1

    return idx
