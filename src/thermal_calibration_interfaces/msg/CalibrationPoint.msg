# A single thermal calibration point
uint32 id                  # Unique identifier for this point
uint16 x                   # X coordinate in the image
uint16 y                   # Y coordinate in the image
uint16 raw_value           # Raw thermal sensor value
float32 reference_temp     # Reference temperature in Celsius
string timestamp           # ISO timestamp when point was captured
