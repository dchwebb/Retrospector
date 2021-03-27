# See http://soundfile.sapp.org/doc/WaveFormat/ for info on wav format

dataSize = 18        # in bytes
samplesPerBit = 4
dataLength = dataSize * 8 * samplesPerBit      # number of bytes, * 8 bits * 4 samples per bit
samplerate = 48000
shiftReg = 0
padding = False

def EncodeByte(b):
    result = bytearray()
    for x in range(8):
        if b == -1:
            result += EncodeBit(-1)
        else:
            if b & 1:
                result += EncodeBit(1)
            else:
                result += EncodeBit(0)
            b = b >> 1

    return result

def EncodeBit(bitVal):
    global shiftReg, padding

    if bitVal < 0:
        padding = True
        return bytearray((0x80808080).to_bytes(4, "little"))

    # If previously padding create transition to bit value
    if padding:
        padding = False
        if bitVal:
            shiftReg = 0b001
        else:
            shiftReg = 0b010
        return bytearray()

    # shift in bit values and write middle bit as pseudo sine when we know the transitions from bit to bit
    shiftReg = ((shiftReg << 1) | bitVal) & 7

    encodedBit = {
        0: 0x00000000,            # 000       
        1: 0x00000055,            # 001
        2: 0xAAFFFFAA,            # 010
        3: 0xAAFFFFFF,            # 011
        4: 0x55000000,            # 100
        5: 0x55000055,            # 101
        6: 0xFFFFFFAA,            # 110
        7: 0xFFFFFFFF             # 111
        }

    return bytearray((encodedBit.get(shiftReg)).to_bytes(4, "big"))



f = open("encoded.wav", "wb")

# 'RIFF'
byte_arr = [0x52, 0x49, 0x46, 0x46]

# ChunkSize: 36 + SubChunk2Size, or more precisely  4 + (8 + SubChunk1Size) + (8 + SubChunk2Size) This is the size of the entire file in bytes minus 8 bytes
byte_arr += (36 + dataLength).to_bytes(4, "little")

byte_arr += [0x57, 0x41, 0x56, 0x45]                # 'WAVE'
byte_arr += [0x66, 0x6d, 0x74, 0x20]                # 'fmt '
byte_arr += [0x10, 0x00, 0x00, 0x00]                # Sub chunk 1 size (ie 16) -effectively  header size
byte_arr += [0x01, 0x00]                            # 1 = PCM format
byte_arr += [0x01, 0x00]                            # Number of channels = 1
byte_arr += (samplerate).to_bytes(4, "little")      # sample rate
byte_arr += (samplerate).to_bytes(4, "little")      # byte rate (ie 1 channels, 8 bit = 1 bytes per sample)
byte_arr += [0x04, 0x00]                            # Block align = 4
byte_arr += [0x08, 0x00]                            # Bits per sample = 8
byte_arr += [0x64, 0x61, 0x74, 0x61]                # 'data'
byte_arr += (dataLength).to_bytes(4, "little")      # subchunk 2 = number of bytes in the data. NumSamples * NumChannels * BitsPerSample/8
byte_arr += EncodeByte(-1)                          # Add padding to start

for x in range(int(dataSize / 4)):
    byte_arr += EncodeByte(0x55)
    byte_arr += EncodeByte(0xAA)
    byte_arr += EncodeByte(0x00)
    byte_arr += EncodeByte(0xFF)

byte_arr += EncodeByte(-1)                          # Add padding to end


binary_format = bytearray(byte_arr)
f.write(binary_format)
f.close()

