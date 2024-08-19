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


## DigiMod Version

# Binary Encoding Protocol for Arduino Communication

## Overview

This repository contains the binary encoding protocol designed for efficient communication between multiple Arduinos and a Linux PC over separate USB connections. The protocol is streamlined to minimize data size and processing time, ensuring fast and reliable message transmission.

## Protocol Structure

The protocol is structured to encode messages into a compact binary format. Each message includes a command ID, command type, and associated data such as model names, libraries, and response data.

### Message Format

Each message follows this structure:

1. **Command ID** (`command_id`): A 32-bit (4-byte) integer that uniquely identifies the command.
2. **Command Type** (`command_type`): An 8-bit (1-byte) value that specifies the type of command (e.g., `init`, `start`, `stop`).
3. **Model Name** (`model`): A dynamic UTF-8 string representing the model name, with a length prefix.
4. **Library Name** (`library`): A dynamic UTF-8 string representing the library name, with a length prefix.
5. **Response Data** (`response_data`): A dynamic payload containing the response, prefixed with a length.

### Encoding Details

- **Command ID**: 4 bytes, unsigned integer.
- **Command Type**: 1 byte.
- **Model Name**:
  - Length Prefix: 1 byte (up to 255 characters).
  - UTF-8 encoded string.
- **Library Name**:
  - Length Prefix: 1 byte (up to 255 characters).
  - UTF-8 encoded string.
- **Response Data**:
  - Length Prefix: 2 bytes (up to 65535 bytes).
  - Raw binary data or UTF-8 encoded string.

### Example Encoding

For a message with the following content:
"command: (id: 1, init(model: VCO, library: Fundamental)); response: (id: 1, null)"

The binary representation would be:

In bytes:
- **Command ID**: `0x00000001` (4 bytes).
- **Command Type**: `0x01` (1 byte).
- **Model Name**: `0x03` + "VCO" (4 bytes).
- **Library Name**: `0x0B` + "Fundamental" (12 bytes).
- **Response Data**: `0x0004` + "null" (6 bytes).

## Usage

### Sending Messages

To send a message using this protocol, encode each field as specified and concatenate the bytes to form the complete binary message. Transmit the message over the serial connection associated with the Arduino.

### Receiving Messages

To receive a message, read the incoming bytes, extract and decode each field according to the protocol structure. Handle the command based on the `command_id` and `command_type`.

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue if you find bugs or have suggestions for improvements.