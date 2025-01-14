import struct

# Constants
PREAMB_CNAV = 0x8B  # Example constant
MAX_SUBFRM_SIZE = 38  # Example size for CNAV/BDS subframes

# Helper functions
def U4(data, offset):
    return struct.unpack_from('<I', data, offset)[0]

def getbitu(buff, pos, length):
    """Extract unsigned bits from buffer"""
    bits = 0
    for i in range(length):
        byte_pos = (pos + i) // 8
        bit_pos = 7 - (pos + i) % 8
        bits = (bits << 1) | ((buff[byte_pos] >> bit_pos) & 1)
    return bits

def setbitu(buff, pos, length, value):
    """Set unsigned bits in buffer"""
    for i in range(length):
        byte_pos = (pos + i) // 8
        bit_pos = 7 - (pos + i) % 8
        if value & (1 << (length - i - 1)):
            buff[byte_pos] |= 1 << bit_pos
        else:
            buff[byte_pos] &= ~(1 << bit_pos)

def time_to_gpst(time):
    """Convert time to GPS seconds"""
    # Add GPS epoch time handling here if needed
    return time

# Decoders
def decode_nav(raw, sat, off):
    """Decode GPS/QZSS navigation data"""
    p = raw[6 + off:] if isinstance(raw, (bytes, bytearray)) else raw['buff'][6 + off:]
    if len(p) < 48:
        print(f"Error: NAV length error: sat={sat}, len={len(p)}")
        return -1

    if (U4(p, 0) >> 24) == PREAMB_CNAV:
        print(f"Unsupported NAV type: sat={sat}")
        return 0

    buff = bytearray(30)
    for i in range(10):
        setbitu(buff, 24 * i, 24, U4(p, i * 4) >> 6)

    id = getbitu(buff, 43, 3)
    if id < 1 or id > 5:
        print(f"Error: NAV subframe ID error: sat={sat}, id={id}")
        return -1

    subfrm = raw.get('subfrm', {})
    subfrm[sat - 1] = subfrm.get(sat - 1, bytearray(150))
    subfrm[sat - 1][(id - 1) * 30:(id - 1) * 30 + 30] = buff

    if id == 3:
        return decode_eph(raw, sat)
    if id in [4, 5]:
        ret = decode_ionutc(raw, sat)
        subfrm[sat - 1][(id - 1) * 30:(id - 1) * 30 + 30] = bytearray(30)
        return ret

    return 0

def decode_enav(raw, sat, off):
    """Decode Galileo I/NAV navigation data"""
    p = raw[6 + off:] if isinstance(raw, (bytes, bytearray)) else raw['buff'][6 + off:]
    if len(p) < 44:
        print(f"Error: ENAV length error: sat={sat}, len={len(p)}")
        return -1

    buff = bytearray(32)
    for i in range(8):
        struct.pack_into('>I', buff, i * 4, U4(p, i * 4))

    part1 = getbitu(buff, 0, 1)
    part2 = getbitu(buff, 128, 1)
    if part1 != 0 or part2 != 1:
        print(f"Page even/odd error: sat={sat}")
        return -1

    type = getbitu(buff, 2, 6)
    if type > 6:
        return 0

    subfrm = raw.get('subfrm', {})
    subfrm[sat - 1] = subfrm.get(sat - 1, bytearray(128))
    subfrm[sat - 1][type * 16:type * 16 + 16] = buff[:16]

    if type != 5:
        return 0

    # Decode Galileo ephemeris (placeholder)
    return 2

def decode_cnav(raw, sat, off):
    """Decode BDS navigation data"""
    p = raw[6 + off:] if isinstance(raw, (bytes, bytearray)) else raw['buff'][6 + off:]
    if len(p) < 48:
        print(f"Error: CNAV length error: sat={sat}, len={len(p)}")
        return -1

    buff = bytearray(MAX_SUBFRM_SIZE)
    for i in range(10):
        setbitu(buff, 30 * i, 30, U4(p, i * 4))

    id = getbitu(buff, 15, 3)
    if id < 1 or id > 5:
        print(f"CNAV subframe ID error: sat={sat}, id={id}")
        return -1

    subfrm = raw.get('subfrm', {})
    subfrm[sat - 1] = subfrm.get(sat - 1, bytearray(MAX_SUBFRM_SIZE * 5))
    subfrm[sat - 1][(id - 1) * MAX_SUBFRM_SIZE:(id - 1) * MAX_SUBFRM_SIZE + MAX_SUBFRM_SIZE] = buff[:MAX_SUBFRM_SIZE]

    return 2

def decode_gnav(raw, sat, off, frq):
    """Decode GLONASS navigation data"""
    p = raw[6 + off:] if isinstance(raw, (bytes, bytearray)) else raw['buff'][6 + off:]
    if len(p) < 24:
        print(f"Error: GNAV length error: sat={sat}, len={len(p)}")
        return -1

    buff = bytearray(64)
    for i in range(4):
        for j in range(4):
            buff[i * 4 + j] = p[i * 4 + (3 - j)]

    # Placeholder for GLONASS decoding logic
    return 2

def decode_snav(raw, prn, off):
    """Decode SBAS navigation data"""
    p = raw[6 + off:] if isinstance(raw, (bytes, bytearray)) else raw['buff'][6 + off:]
    if len(p) < 40:
        print(f"Error: SNAV length error: len={len(p)}")
        return -1

    buff = bytearray(32)
    for i in range(8):
        struct.pack_into('>I', buff, i * 4, U4(p, i * 4))

    raw['sbsmsg'] = {
        'prn': prn,
        'tow': int(time_to_gpst(raw['time'])) % 604800,
        'week': int(time_to_gpst(raw['time'])) // 604800,
        'msg': buff[:29],
    }
    return 3


# Define constants for GNSS systems
SYS_GPS = 1
SYS_QZS = 2
SYS_GAL = 3
SYS_CMP = 4
SYS_GLO = 5
SYS_SBS = 6

def ubx_sys(gnss_id):
    """Convert GNSS ID to system constant."""
    if gnss_id == 0: return SYS_GPS
    if gnss_id == 1: return SYS_SBS
    if gnss_id == 2: return SYS_GAL
    if gnss_id == 3: return SYS_GLO
    if gnss_id == 5: return SYS_QZS
    if gnss_id == 6: return SYS_CMP
    return 0  # Unknown system

def satno(sys, prn):
    """Convert system and PRN to satellite number."""
    if sys == SYS_GPS or sys == SYS_QZS:
        if 1 <= prn <= 32: return prn
        if sys == SYS_QZS and 193 <= prn <= 202: return prn - 192
    elif sys == SYS_GLO:
        if 1 <= prn <= 24: return prn
    elif sys == SYS_GAL:
        if 1 <= prn <= 36: return prn
    elif sys == SYS_CMP:
        if 1 <= prn <= 63: return prn
    elif sys == SYS_SBS:
        if 120 <= prn <= 158: return prn
    return 0  # Invalid satellite

def decode_rxmsfrbx(raw):
    """Decode UBX-RXM-SFRBX: raw subframe data."""
    if len(raw) < 8:
        print("Error: Insufficient data length")
        return None

    p = raw[6:]  # Skip header
    gnss_id = p[0]  # GNSS ID
    prn = p[1]  # PRN number

    # Determine GNSS system
    sys = ubx_sys(gnss_id)
    if sys == 0:
        print(f"Error: Unknown GNSS system ID {gnss_id}")
        return None

    # Adjust PRN for QZSS
    if sys == SYS_QZS:
        prn += 192

    # Get satellite number
    sat = satno(sys, prn)
    if sat == 0:
        if sys == SYS_GLO and prn == 255:
            print("Warning: Unknown GLONASS satellite, skipping")
            return None
        print(f"Error: Invalid satellite number for sys={sys}, prn={prn}")
        return None

    # Handle QZSS L1S signal
    if sys == SYS_QZS and len(raw) == 52:
        sys = SYS_SBS
        prn -= 10

    # Decode subframe data (based on GNSS system type)
    payload_data = p[8:]  # Subframe data starts at offset 8
    print(f"Decoding subframe data for sys={sys}, prn={prn}, sat={sat}")

    if sys == SYS_GPS or sys == SYS_QZS:
        subframe = decode_nav(raw, sat, 8)
    elif sys == SYS_GAL:
        subframe = decode_enav(raw, sat, 8)
    elif sys == SYS_CMP:
        subframe = decode_cnav(raw, sat, 8)
    elif sys == SYS_GLO:
        frq = p[3]  # Frequency number for GLONASS
        subframe = decode_gnav(raw, sat, 8, frq)
    elif sys == SYS_SBS:
        subframe = decode_snav(raw, prn, 8)
    else:
        print(f"Error: Unsupported GNSS system {sys}")
        return None

    return {
        "system": sys,
        "prn": prn,
        "satellite": sat,
        "subframe": subframe,
    }


def decode_subframe(sys, sat, payload):
    """Example decoder for subframe data."""
    if len(payload) < 8:
        print("Error: Insufficient subframe data length")
        return None

    # Example: extract first 8 bytes as subframe data (adjust as needed)
    subframe = struct.unpack('<8B', payload[:8])
    print(f"Subframe data (first 8 bytes): {subframe}")
    return subframe

# Example raw data (hexadecimal string converted to bytes)
raw_data = bytes.fromhex(
    "B5 62 02 13 2C 00 01 80 00 00 09 26 02 70 00 01 FC 9A 67 B2 10 00 20 00 00 E9 00 00 14 C0 00 00 00 00 00 00 00 00 00 00 00 00 00 BC 50 1B 00 00 00 00 27 26"
)

# Decode the SFRBX message
result = decode_rxmsfrbx(raw_data)
print("Decoded SFRBX message:", result)
