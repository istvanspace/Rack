# VCV Rack

*Rack* is the host application for the VCV virtual Eurorack modular synthesizer platform.

- [VCV website](https://vcvrack.com/)
- [Manual](https://vcvrack.com/manual/)
- [Support](https://vcvrack.com/support)
- [Module Library](https://library.vcvrack.com/)
- [Rack source code](https://github.com/VCVRack/Rack)
- [Building](https://vcvrack.com/manual/Building)
- [Communities](https://vcvrack.com/manual/Communities)
- [Licenses](LICENSE.md) ([HTML](LICENSE.html))

## Acknowledgments

- [Andrew Belt](https://github.com/AndrewBelt): Lead Rack developer
- [Pyer](https://www.pyer.be/): Module design, component graphics
- [Richie Hindle](http://entrian.com/audio/): Rack developer, bug fixes
- [Grayscale](https://grayscale.info/): Module design, branding
- Christoph Scholtes: [Library reviews](https://github.com/VCVRack/library) and [plugin toolchain](https://github.com/VCVRack/rack-plugin-toolchain)
- Rack plugin developers: Authorship shown on each plugin's [VCV Library](https://library.vcvrack.com/) page
- Rack users like you: [Bug reports and feature requests](https://vcvrack.com/support)

## Dependency libraries

- [GLFW](https://www.glfw.org/)
- [GLEW](http://glew.sourceforge.net/)
- [NanoVG](https://github.com/memononen/nanovg)
- [NanoSVG](https://github.com/memononen/nanosvg)
- [oui-blendish](https://github.com/geetrepo/oui-blendish)
- [osdialog](https://github.com/AndrewBelt/osdialog) (written by Andrew Belt for VCV Rack)
- [ghc::filesystem](https://github.com/gulrak/filesystem)
- [Jansson](https://digip.org/jansson/)
- [libcurl](https://curl.se/libcurl/)
- [OpenSSL](https://www.openssl.org/)
- [Zstandard](https://facebook.github.io/zstd/) (for Rack's `.tar.zstd` patch format)
- [libarchive](https://libarchive.org/) (for Rack's `.tar.zstd` patch format)
- [PFFFT](https://bitbucket.org/jpommier/pffft/)
- [libspeexdsp](https://gitlab.xiph.org/xiph/speexdsp/-/tree/master/libspeexdsp) (for Rack's fixed-ratio resampler)
- [libsamplerate](https://github.com/libsndfile/libsamplerate) (for Rack's variable-ratio resampler)
- [RtMidi](https://www.music.mcgill.ca/~gary/rtmidi/)
- [RtAudio](https://www.music.mcgill.ca/~gary/rtaudio/)
- [Fuzzy Search Database](https://bitbucket.org/j_norberg/fuzzysearchdatabase) (written by Nils Jonas Norberg for VCV Rack's module browser)
- [TinyExpr](https://codeplea.com/tinyexpr) (for math evaluation in parameter context menu)

## Contributions

VCV cannot accept free contributions to Rack itself, but we encourage you to

- Send us feature requests and bug reports.
- Create a plugin that extends Rack's functionality. Most of Rack's functionality is exposed in its public plugin API.
- Work at VCV! Check job openings at <https://vcvrack.com/jobs>


# Binary Encoding Protocol for Arduino Communication

## DigiMod Version

## Overview
This repository contains the binary encoding protocol designed for efficient communication between multiple Arduinos and a Linux PC over separate USB connections. The protocol is streamlined to minimize data size and processing time, ensuring fast and reliable message transmission.

## Protocol Structure
The protocol is structured to encode messages into a compact binary format. Each message includes start and end markers, message length, command ID, command type, and associated payload data.

### Message Format
Each message follows this structure:

1. **Start Marker** (`START_MARKER`): A 1-byte value (0xFE) indicating the start of a message.
2. **Message Length** (`message_length`): A 2-byte value indicating the length of the message (excluding start/end markers and length itself).
3. **Command ID** (`command_id`): A 1-byte value that uniquely identifies the command.
4. **Command Type** (`command_type`): A 1-byte value that specifies the type of command (e.g., `init`, `start`, `stop`).
5. **Payload Length 1** (`payload1_length`): A 1-byte value indicating the length of the first payload.
6. **Payload Length 2** (`payload2_length`): A 1-byte value indicating the length of the second payload.
7. **Payload 1**: Variable length data (e.g., model name, or parameter_id).
8. **Payload 2**: Variable length data (e.g., library name, or parameter_value).
9. **End Marker** (`END_MARKER`): A 1-byte value (0xFF) indicating the end of a message.

### Encoding Details
- **Start Marker**: 1 byte, fixed value 0xFE.
- **Message Length**: 2 bytes, unsigned integer.
- **Command ID**: 1 byte, unsigned integer.
- **Command Type**: 1 byte.
- **Payload Length 1**: 1 byte (up to 255 characters).
- **Payload Length 2**: 1 byte (up to 255 characters).
- **Payload 1**: Variable length, UTF-8 encoded string or raw data.
- **Payload 2**: Variable length, UTF-8 encoded string or raw data.
- **End Marker**: 1 byte, fixed value 0xFF.

### Example Encoding
For a message with the following content:
"command: (id: 1, init(model: VCO, library: Fundamental))"

The binary representation would be:

```
FE 00 12 01 01 03 0B 56 43 4F 46 75 6E 64 61 6D 65 6E 74 61 6C FF
```

Breakdown:
- `FE`: Start Marker
- `00 12`: Message Length (18 bytes)
- `01`: Command ID
- `01`: Command Type (init)
- `03`: Payload 1 Length (3 bytes)
- `0B`: Payload 2 Length (11 bytes)
- `56 43 4F`: Payload 1 ("VCO")
- `46 75 6E 64 61 6D 65 6E 74 61 6C`: Payload 2 ("Fundamental")
- `FF`: End Marker

## Usage

### Sending Messages
To send a message using this protocol:
1. Construct the message according to the format above.
2. Calculate the message length (excluding start/end markers and length itself).
3. Encode each field as specified.
4. Concatenate the bytes to form the complete binary message.
5. Transmit the message over the serial connection.

### Receiving Messages
To receive a message:
1. Read incoming bytes until the Start Marker (0xFE) is encountered.
2. Read the next 2 bytes to determine the message length.
3. Read the specified number of bytes.
4. Verify the End Marker (0xFF).
5. Decode the message fields according to the protocol structure.
6. Handle the command based on the `command_id` and `command_type`.

## Contributing
Contributions are welcome! Please feel free to submit a pull request or open an issue if you find bugs or have suggestions for improvements.