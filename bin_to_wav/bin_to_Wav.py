f = open("encoded.wav", "wb")

# See http://soundfile.sapp.org/doc/WaveFormat/ for info on wav format

# RIFF
byte_arr = [0x52, 0x49, 0x46, 0x46]

# ChunkSize: 36 + SubChunk2Size, or more precisely  4 + (8 + SubChunk1Size) + (8 + SubChunk2Size)
# This is the size of the rest of the chunk following this number.  This is the size of the entire file in bytes minus 8 bytes
# ie 0x0824
byte_arr += [0x24, 0x08, 0x00, 0x00]

# WAVEfmt_
byte_arr += [0x57, 0x41, 0x56, 0x45, 0x66, 0x6d, 0x74, 0x20]

# Sub chunk 1 size (ie 16) -effectively  header size
byte_arr += [0x10, 0x00, 0x00, 0x00]

# PCM format
byte_arr += [0x01, 0x00]

# Number of channels
byte_arr += [0x01, 0x00]

# sample rate 0x5622 = 22050
byte_arr += [0x22, 0x56, 0x00, 0x00]

# byte rate 0x15888 = 88,200 (ie 2 channels, 16 bits = 4 bytes per sample)
byte_arr += [0x88, 0x58, 0x01, 0x00]

# Block align = 4
byte_arr += [0x04, 0x00]

# Bits per sample = 16
byte_arr += [0x10, 0x00]

# data
byte_arr += [0x64, 0x61, 0x74, 0x61]

# subchunk 2 size = 2048. Effectively data size = NumSamples * NumChannels * BitsPerSample/8
byte_arr += [0x00, 0x08, 0x00, 0x00]

binary_format = bytearray(byte_arr)
f.write(binary_format)
f.close()

