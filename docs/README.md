# Rust no_std LilyGo T5 v2.3.1 2.13

## Main example with WiFi support

no_std example for e-ink LilyGoT5 with WiFi support.
The example connects to an IP address and download text file.
Then the content of the text is displayed on the screen.

Build and flash:

```
export SSID="Wokwi-GUEST"
export PASSWORD=""
cargo run --release
```

## Hello example

Simpler example which just draws to the display

```
cargo run --release --example hello
```

### Wokwi simulation

Once the release build is ready, it's possible to launch simulator.
Go to VS Code and open Wokwi simulation.
