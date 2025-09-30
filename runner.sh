# Execute probe-rs runner with the binary path
exec probe-rs run --chip RP2040 --log-format "[{t:blue}] {f:>10} {L} {s}" "$@"
