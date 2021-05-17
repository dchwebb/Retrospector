# See http://soundfile.sapp.org/doc/WaveFormat/ for info on wav format

sampleRate = 48000
sampleArray = []
headerArray = []
binFilePath = "D:\Eurorack\Retrospector\Bootloader\Debug\Bootloader.bin"
#binFilePath = "D:\Eurorack\Retrospector\Retrospector_H743_v2\Release\Retrospector_H743_v2.bin"
checksumFreq = 500                              # Insert checksum every x bytes
versionMajor = 2
versionMinor = 1

# Encodes byte into sequence of four samples, creating smoothing transitions as required
# pass -1 to output 8 bits at the mid-value for padding
def EncodeByte(b):
    global sampleArray

    for x in range(8):
        lastByte = len(sampleArray) - 1
        if b == -1:
            sampleArray += bytearray((0x80808080).to_bytes(4, "little"))
            # Create transitions if necessary
            if lastByte > 44:
                if sampleArray[lastByte] == 0x00:
                    sampleArray[lastByte] = 0x55
                if sampleArray[lastByte] == 0xFF:
                    sampleArray[lastByte] = 0xAA
        else:
            if b & 0x80:
                sampleArray += bytearray((0xFFFFFFFF).to_bytes(4, "little"))
                if sampleArray[lastByte] == 0x00:            # transitioning to 1 from 0
                    sampleArray[lastByte] = 0x55
                if sampleArray[lastByte] < 0xFF:             # transitioning to 1 from mid point or 0
                    sampleArray[lastByte + 1] = 0xAA
            else:
                sampleArray += bytearray((0x00000000).to_bytes(4, "little"))
                if sampleArray[lastByte] == 0xFF:            # transitioning to 0 from 1
                    sampleArray[lastByte] = 0xAA
                if sampleArray[lastByte] > 0x00:             # transitioning to 0 from mid point or 1
                    sampleArray[lastByte + 1] = 0x55

            b = b << 1

    return


wavFile = open("encoded.wav", "wb")
binFile = open(binFilePath, "rb")
binFile.seek(0, 2)                                     # move the cursor to the end of the file
rawDataSize = binFile.tell()                           # Get the file size
checkSumSize = rawDataSize // checksumFreq             # Floor division


EncodeByte(-1)                                         # Add padding to start
EncodeByte(0xAA)                                       # Add timing pattern to start
EncodeByte(0xAA)
EncodeByte(0x00)
EncodeByte(0xAA)
EncodeByte((rawDataSize >> 24) & 0xFF)                 # Next 4 bytes are the file size
EncodeByte((rawDataSize >> 16) & 0xFF)
EncodeByte((rawDataSize >>  8) & 0xFF)
EncodeByte((rawDataSize >>  0) & 0xFF)
EncodeByte(versionMajor)
EncodeByte(versionMinor)

checkSum = 0
cs_arr = []

binFile.seek(0, 0)                                     # Go to beginning of binary file
for x in range(rawDataSize):
    nextByte = int.from_bytes(binFile.read(1), "little")
    nextByte = nextByte ^ 0xCC                         # XOR byte with 0b11001100 to ensure clock recovery transitions

    if binFile.tell() % checksumFreq == 0 and binFile.tell() > 0:
        EncodeByte(checkSum & 0xFF)                    # Insert checksum
        cs_arr += [checkSum & 0xFF]
        checkSum = 0

    EncodeByte(nextByte)
    checkSum += nextByte

EncodeByte(checkSum & 0xFF)                            # Insert final checksum
EncodeByte(-1)                                         # Add padding to end


# Generate wav header
headerArray += [0x52, 0x49, 0x46, 0x46]                # 'RIFF'

# ChunkSize: This is the size of the entire file in bytes minus 8 bytes
headerArray += (36 + len(sampleArray)).to_bytes(4, "little")

headerArray += [0x57, 0x41, 0x56, 0x45]                # 'WAVE'
headerArray += [0x66, 0x6d, 0x74, 0x20]                # 'fmt '
headerArray += [0x10, 0x00, 0x00, 0x00]                # Sub chunk 1 size (ie 16) -effectively  header size
headerArray += [0x01, 0x00]                            # 1 = PCM format
headerArray += [0x01, 0x00]                            # Number of channels = 1
headerArray += (sampleRate).to_bytes(4, "little")      # sample rate
headerArray += (sampleRate).to_bytes(4, "little")      # byte rate (ie 1 channels, 8 bit = 1 bytes per sample)
headerArray += [0x04, 0x00]                            # Block align = 4
headerArray += [0x08, 0x00]                            # Bits per sample = 8
headerArray += [0x64, 0x61, 0x74, 0x61]                # 'data'
headerArray += (len(sampleArray)).to_bytes(4, "little")# subchunk 2 = number of bytes in the data. NumSamples * NumChannels * BitsPerSample/8

binary_format = bytearray(headerArray + sampleArray)
wavFile.write(binary_format)
wavFile.close()
binFile.close()
