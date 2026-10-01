# Implementation obstacles

## Configured mapping ranges and register precision

**ISSUE:** Startup configuration accepts the existing f64 mapping-range type,
but the emulated PMAX/VMAX/TMAX registers return f32. A value such as 0.1 could
otherwise produce different scaling in the controller and a client that reads
its registers.

**SOLUTION DIRECTIONS:** Reject values not exactly representable in f32, change
the reusable codec's types, or normalize initial register values to f32.

**MOTIVATION FOR CHOSEN OPTION:** Normalize once when constructing the controller,
matching CAN register writes. This preserves the codec API and accepts ordinary
configuration values while keeping reported registers and actual scaling equal.
