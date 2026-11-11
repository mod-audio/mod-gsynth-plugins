# MOD GSynth plugins

Guitar and bass synthesizer plugins for MOD devices, made from MOD Audio's GSYNTH pedalboards: each plugin turns
the notes you play into a synth voice, with the pedalboard's whole signal chain inside one plugin.

| Folder | Plugin | URI |
| --- | --- | --- |
| `mod-gsynth/` | GSynth | `https://mod.audio/plugins/gsynth` |
| `mod-gsynth-flexible/` | GSynth Flexible | `https://mod.audio/plugins/gsynth-flexible` |
| `mod-gsynth-acid-bass/` | GSynth Acid Bass | `https://mod.audio/plugins/gsynth-acid-bass` |
| `mod-gsynth-minor-dub/` | GSynth Minor Dub | `https://mod.audio/plugins/gsynth-minor-dub` |
| `mod-gsynth-bassline/` | GSynth Bassline | `https://mod.audio/plugins/gsynth-bassline` |

Each folder is complete on its own: source, LV2 metadata, presets and interface. Its README lists the controls and
credits the authors of the original plugins its DSP blocks are ported from. The folders are generated from MOD's
synth specifications; changes are made there and exported here.

## Building

Each folder builds alone; it needs the LV2 headers and single-precision FFTW (`fftw3f`):

```sh
cmake -S mod-gsynth-minor-dub -B build && cmake --build build
```

## License

GPL-3.0-or-later (`LICENSE`). The DSP blocks are ports of GPL plugins; each folder's README lists them.
