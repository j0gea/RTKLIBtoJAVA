import struct
import math
from datetime import datetime, timedelta

# Define constants
CLIGHT = 299792458.0  # Speed of light in m/s
MAXOBS = 32
CPSTD_VALID = 10
SNR_UNIT = 4.0

def decode_rxmrawx(raw):
    def R8(data, offset):
        return struct.unpack('<d', data[offset:offset + 8])[0]
    
    def R4(data, offset):
        return struct.unpack('<f', data[offset:offset + 4])[0]
    
    def U2(data, offset):
        return struct.unpack('<H', data[offset:offset + 2])[0]
    
    def U1(data, offset):
        return struct.unpack('<B', data[offset:offset + 1])[0]
    
    def gpst2time(week, tow):
        epoch = datetime(1980, 1, 6)  # GPS epoch
        return epoch + timedelta(weeks=week, seconds=tow)
    
    p = raw[6:]
    
    if len(raw) < 24:
        print(f"Error: Invalid raw data length: {len(raw)}")
        return None

    tow = R8(p, 0)  # Receiver time of week (s)
    week = U2(p, 8)  # GPS week number
    nmeas = U1(p, 11)  # Number of measurements
    ver = U1(p, 13)  # Version

    if len(raw) < 24 + 32 * nmeas:
        print(f"Error: Invalid length for nmeas={nmeas}")
        return None
    
    if week == 0:
        print("Error: GPS week is 0")
        return None

    time = gpst2time(week, tow)

    results = []
    for i in range(nmeas):
        offset = 16 + i * 32
        P = R8(p, offset)  # Pseudorange (m)
        L = R8(p, offset + 8)  # Carrier phase (cycles)
        D = R4(p, offset + 16)  # Doppler (Hz)
        gnss = U1(p, offset + 20)  # GNSS ID
        svid = U1(p, offset + 21)  # Satellite ID
        sigid = U1(p, offset + 22)  # Signal ID
        frqid = U1(p, offset + 23)  # Frequency ID
        lockt = U2(p, offset + 24)  # Lock time (ms)
        cn0 = U1(p, offset + 26)  # Signal-to-noise ratio (dBHz)
        cpstd = U1(p, offset + 28) & 15  # Carrier phase standard deviation
        tstat = U1(p, offset + 30)  # Tracking status

        # Check for valid data
        if not (tstat & 1):  # Pseudorange valid
            P = 0.0
        if not (tstat & 2) or L == -0.5 or cpstd > CPSTD_VALID:  # Carrier phase valid
            L = 0.0

        # Append results
        results.append({
            "time": time,
            "Pseudorange (P)": P,
            "Carrier phase (L)": L,
            "Doppler (D)": D,
            "GNSS ID": gnss,
            "Satellite ID": svid,
            "Signal ID": sigid,
            "Frequency ID": frqid,
            "Lock time (ms)": lockt,
            "SNR (dBHz)": cn0,
            "Carrier phase std dev": cpstd,
            "Tracking status": tstat
        })

    return(results)

a = decode_rxmrawx(bytes.fromhex("B5 62 02 15 50 00 00 00 00 00 38 3F 08 41 2D 09 12 02 01 01 DC 69 CF 62 32 CF E6 9E 72 41 44 18 96 F4 B0 E4 98 41 77 AE 00 C5 06 18 00 09 00 00 0F 0A 0F 0D 01 00 1B BA 49 57 01 A6 75 41 B7 DC 66 EA 7F E6 9C 41 20 22 84 C5 06 0D 00 05 00 00  0B 0B 0F 0D 01 00 88 72"))
print(a)
