# See http://soundfile.sapp.org/doc/WaveFormat/ for info on wav format

dataSize = 18        # in bytes
samplesPerBit = 4
#dataLength = dataSize * 8 * samplesPerBit      # number of bytes, * 8 bits * 4 samples per bit
dataLength = 0
samplerate = 48000
fileByteCounter = 0                            # Count to establish where to place checksums
byte_arr = []
headerArray = []
binFilePath = "D:\CubeIDE\Boottest_H743\Debug\Boottest_H743.bin"
checksumFreq = 100                              # Insert checksum every x bytes
encodingHeader = True

# Encodes byte into sequence of four samples, creating smoothing transitions as required
# pass -1 to output 8 bits at the mid-value for padding
def EncodeByte(b):
    global byte_arr, dataLength, bitStuffCount

    dataLength += (8 * 4)                       # increase size of data for later insertion into wav header
    
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
                bitStuffCount = 0
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
#paddingBytes = 10
checkSumSize = rawDataSize // checksumFreq          # Floor division
#dataLength = (rawDataSize + paddingBytes + checkSumSize) * 8 * 4   # file size plus padding plus check sums



EncodeByte(-1)                                      # Add padding to start
EncodeByte(0xAA)                                    # Add timing pattern to start
EncodeByte(0xAA)
EncodeByte(0x00)
EncodeByte(0xAA)
EncodeByte((rawDataSize >> 24) & 0xFF)              # Next 4 bytes are the file size
EncodeByte((rawDataSize >> 16) & 0xFF)
EncodeByte((rawDataSize >>  8) & 0xFF)
EncodeByte((rawDataSize >>  0) & 0xFF)
encodingHeader = False                              # Allow bit stuffing from this point on
checkSum = 0
cs_arr = []

binFile.seek(0, 0)                                  # Go to beginning of binary file
for x in range(rawDataSize):
    nextByte = int.from_bytes(binFile.read(1), "little")
    
    #rem = x % 8
    #if rem == 0:
    #    nextByte = 0xFD
    #elif rem == 1:
    #    nextByte = 0x04
    #elif rem == 2:
    #    nextByte = 0x10
    #elif rem == 3:
    #    nextByte = 0x08
    #elif rem == 4:
    #    nextByte = 0x00
    #elif rem == 5:
    #    nextByte = 0x00
    #elif rem == 6:
    #    nextByte = 0xAA
    #elif rem == 7:
    #    nextByte = 0x00

    nextByte = nextByte ^ 0xCC

    if binFile.tell() % checksumFreq == 0 and binFile.tell() > 0:
        EncodeByte(checkSum & 0xFF)                 # Insert checksum
        cs_arr += [checkSum & 0xFF]
        checkSum = 0

    EncodeByte(nextByte)
    checkSum += nextByte

EncodeByte(-1)                                      # Add padding to end


# Generate wav header
# 'RIFF'
headerArray += [0x52, 0x49, 0x46, 0x46]

# ChunkSize: 36 + SubChunk2Size, or more precisely  4 + (8 + SubChunk1Size) + (8 + SubChunk2Size) This is the size of the entire file in bytes minus 8 bytes
headerArray += (36 + dataLength).to_bytes(4, "little")

headerArray += [0x57, 0x41, 0x56, 0x45]                # 'WAVE'
headerArray += [0x66, 0x6d, 0x74, 0x20]                # 'fmt '
headerArray += [0x10, 0x00, 0x00, 0x00]                # Sub chunk 1 size (ie 16) -effectively  header size
headerArray += [0x01, 0x00]                            # 1 = PCM format
headerArray += [0x01, 0x00]                            # Number of channels = 1
headerArray += (samplerate).to_bytes(4, "little")      # sample rate
headerArray += (samplerate).to_bytes(4, "little")      # byte rate (ie 1 channels, 8 bit = 1 bytes per sample)
headerArray += [0x04, 0x00]                            # Block align = 4
headerArray += [0x08, 0x00]                            # Bits per sample = 8
headerArray += [0x64, 0x61, 0x74, 0x61]                # 'data'
headerArray += (dataLength).to_bytes(4, "little")      # subchunk 2 = number of bytes in the data. NumSamples * NumChannels * BitsPerSample/8


binary_format = bytearray(headerArray + byte_arr)
wavFile.write(binary_format)
wavFile.close()
binFile.close()
