# See http://soundfile.sapp.org/doc/WaveFormat/ for info on wav format

dataSize = 18        # in bytes
samplesPerBit = 4
dataLength = dataSize * 8 * samplesPerBit      # number of bytes, * 8 bits * 4 samples per bit
samplerate = 48000
shiftReg = 0
padding = False
byte_arr = []
binFilePath = "D:\CubeIDE\Boottest_H743\Debug\Boottest_H743.bin"

# Encodes byte into sequence of four samples, creating smoothing transitions as required
# pass -1 to output 8 bits at the mid-value for padding
def EncodeByte(b):
    global byte_arr
    
    for x in range(8):
        lastByte = len(byte_arr) - 1
        if b == -1:
            byte_arr += bytearray((0x80808080).to_bytes(4, "little"))
            # Create transitions if necessary
            if lastByte > 44:
                if byte_arr[lastByte] == 0x00:
                    byte_arr[lastByte] = 0x55
                if byte_arr[lastByte] == 0xFF:
                    byte_arr[lastByte] = 0xAA
        else:
            if b & 0x80:
                byte_arr += bytearray((0xFFFFFFFF).to_bytes(4, "little"))
                if byte_arr[lastByte] == 0x00:            # transitioning to 1 from 0
                    byte_arr[lastByte] = 0x55
                if byte_arr[lastByte] < 0xFF:             # transitioning to 1 from mid point or 0
                    byte_arr[lastByte + 1] = 0xAA
            else:
                byte_arr += bytearray((0x00000000).to_bytes(4, "little"))
                if byte_arr[lastByte] == 0xFF:            # transitioning to 0 from 1
                    byte_arr[lastByte] = 0xAA
                if byte_arr[lastByte] > 0x00:             # transitioning to 0 from mid point or 1
                    byte_arr[lastByte + 1] = 0x55

            b = b << 1

    return


wavFile = open("encoded.wav", "wb")
binFile = open(binFilePath, "rb")
binFile.seek(0, 2)                                  # move the cursor to the end of the file
rawDataSize = binFile.tell()
paddingBytes = 4
dataLength = (rawDataSize + paddingBytes) * 8 * 4              # file size plus padding


# Generate wav header
# 'RIFF'
byte_arr += [0x52, 0x49, 0x46, 0x46]

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

EncodeByte(-1)                                      # Add padding to start
EncodeByte(0xAA)                                    # Add 10 timing pattern to start
EncodeByte(0xAA)
binFile.seek(0, 0)                                  # Go to beginning of binary file

for x in range(rawDataSize):
    EncodeByte(int.from_bytes(binFile.read(1), "little"))

EncodeByte(-1)                                       # Add padding to end


binary_format = bytearray(byte_arr)
wavFile.write(binary_format)
wavFile.close()
binFile.close()
