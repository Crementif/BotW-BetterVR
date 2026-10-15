#pragma once

namespace Guest {
    inline uint64_t s_baseAddress = 0;

    template <typename T = char>
    T* HostPointer(uint32_t address) {
        return (T*)(s_baseAddress + address);
    }

    inline const char* CString(uint32_t address) {
        return HostPointer<const char>(address);
    }

    inline std::string String(uint32_t address) {
        return address == 0 ? std::string() : std::string(CString(address));
    }

    template <typename T>
    void Read(uint32_t address, T* out) {
        memcpy(out, HostPointer<void>(address), sizeof(T));
    }

    template <typename T>
    void ReadBE(uint32_t address, T* out) {
        Read(address, out);
        *out = swapEndianness(*out);
    }

    template <typename T>
    void Write(uint32_t address, const T* value) {
        memcpy(HostPointer<void>(address), value, sizeof(T));
    }

    template <typename T>
    void WriteBE(uint32_t address, const T* value) {
        const T swapped = swapEndianness(*value);
        Write(address, &swapped);
    }

    template <typename T>
    auto Get(uint32_t address) {
        if constexpr (is_BEType_v<T>) {
            T result;
            Read(address, &result);
            return result;
        }
        else {
            BEType<T> result;
            Read(address, &result);
            return result;
        }
    }

    template <typename T>
    void Set(uint32_t address, T value) {
        if constexpr (is_BEType_v<T>) {
            Write(address, &value);
        }
        else {
            BEType<T> beValue = value;
            Write(address, &beValue);
        }
    }
}
