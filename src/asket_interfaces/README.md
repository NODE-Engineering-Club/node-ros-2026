# asket_interfaces

Message and service definitions shared by `asket_sim`, `omniscan_bridge`,
`mission_recorder`, `system_test` and `gui_backend`.

Nothing here depends on the packages that use it, and nothing here is generated
at runtime — a message change is a rebuild.

## Testing in sim

There is nothing to test: these are declarations. What *is* worth checking is
that the field set still matches what `gui_backend` puts on the wire. The
adapters in `gui_backend/core/adapters.py` are the single place both sides meet,
and `pytest src/gui_backend` exercises them against the simulated sources
without ROS.

## A note on `PicoStatus`

This is **our** message, published by `asket_sim`. The real `pico_bridge`
message is not known in this repository and `pico_bridge` must not be modified
(see `docs/open_questions.md` Q7). `gui_backend` never subscribes to a message
type directly — it goes through `config/topics.yaml`, so pointing it at the real
type is a config change plus one adapter function.
